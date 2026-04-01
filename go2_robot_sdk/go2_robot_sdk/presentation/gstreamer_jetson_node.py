import cv2
import rclpy
import os
import threading
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

os.environ['LD_PRELOAD'] = '/usr/lib/aarch64-linux-gnu/libjpeg.so.8'

class Go2GstreamerJetsonNode(Node):

    def __init__(self):
        super().__init__('go2_gstreamer_jetson_node')

        self.declare_parameter('image_topic', '/frontcamera/compressed')
        self.declare_parameter('frame_id', 'go2_front_camera')
        self.declare_parameter('multicast_address', '230.1.1.1')
        self.declare_parameter('port', 1720)
        self.declare_parameter('multicast_iface', 'enP8p1s0') # Interface sesuai tes CLI Anda
        self.declare_parameter('buffer_size', 524288)
        self.declare_parameter('latency_ms', 10)       # turun dari 40 → kurangi buffer jitter
        self.declare_parameter('timer_period', 0.015)  # ~66 Hz polling, turun dari 0.03
        self.declare_parameter('jpeg_quality', 60)     # turun dari 90 → encoding jauh lebih cepat
        self.declare_parameter('output_width', 480)    # turun dari 640
        self.declare_parameter('output_height', 270)   # turun dari 360
        self.declare_parameter('output_fps', 20)      
        self.declare_parameter('pipeline', '')

        image_topic = str(self.get_parameter('image_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.output_width = int(self.get_parameter('output_width').value)
        self.output_height = int(self.get_parameter('output_height').value)
        self.output_fps = int(self.get_parameter('output_fps').value)
        self.min_publish_period_ns = int(1e9 / self.output_fps) if self.output_fps > 0 else 0
        self.last_publish_ns = 0
        timer_period = float(self.get_parameter('timer_period').value)

        self.pub = self.create_publisher(CompressedImage, image_topic,3)  # queue=1, buang frame lama

        self.pipeline = str(self.get_parameter('pipeline').value).strip() or self._build_pipeline()
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open GStreamer pipeline: {self.pipeline}')
            raise RuntimeError('GStreamer pipeline could not be opened')

        # Thread terpisah membaca frame terus-menerus agar cap.read() tidak
        # memblokir timer callback ROS → latensi lebih rendah
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.timer = self.create_timer(timer_period, self.loop)
        self.get_logger().info(f'GStreamer Jetson node started, publishing to {image_topic}')

    def _build_pipeline(self):
        m_addr = str(self.get_parameter('multicast_address').value)
        port = int(self.get_parameter('port').value)
        iface = str(self.get_parameter('multicast_iface').value)
        buf_size = int(self.get_parameter('buffer_size').value)
        latency = int(self.get_parameter('latency_ms').value)

        return (
            f'udpsrc address={m_addr} port={port} multicast-iface={iface} buffer-size={buf_size} '
            '! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 '
            f'! rtpjitterbuffer latency={latency} drop-on-latency=true '
            '! rtph264depay '
            '! h264parse config-interval=-1 '
            '! nvv4l2decoder enable-max-performance=1 '
            '! nvvidconv '
            '! video/x-raw, format=BGRx '
            '! videoconvert '
            '! video/x-raw, format=BGR '
            '! appsink drop=true max-buffers=1 sync=false emit-signals=false'
        )

    def _reader_loop(self):
        """Baca frame dari GStreamer secepat mungkin di thread latar."""
        while self._running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self._frame_lock:
                    self._latest_frame = frame

    def loop(self):
        # Ambil frame terbaru tanpa blocking
        with self._frame_lock:
            frame = self._latest_frame
            self._latest_frame = None  # konsumsi → hindari publish frame duplikat

        if frame is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.min_publish_period_ns > 0 and self.last_publish_ns > 0:
            if now_ns - self.last_publish_ns < self.min_publish_period_ns:
                return

        if self.output_width > 0 and self.output_height > 0:
            # INTER_LINEAR ~2× lebih cepat dari INTER_AREA, kualitas cukup
            frame = cv2.resize(frame, (self.output_width, self.output_height),
                               interpolation=cv2.INTER_LINEAR)

        encoded, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])

        if encoded:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = jpeg.tobytes()

            self.pub.publish(msg)
            self.last_publish_ns = now_ns

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_reader_thread'):
            self._reader_thread.join(timeout=1.0)
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Go2GstreamerJetsonNode()
        rclpy.spin(node)
    except Exception as exc:
        if node: node.get_logger().error(f'Node error: {exc}')
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
