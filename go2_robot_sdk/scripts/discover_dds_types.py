#!/usr/bin/env python3
"""
Discover actual DDS topic names and typenames published by the robot.
This helps identify the correct typename to use in DDSBridge.
"""

from cyclonedds.domain import DomainParticipant
from cyclonedds.builtin import BuiltinTopicDcpsPublication
from cyclonedds.sub import DataReader
import time

def discover_topics(duration=5):
    """Discover all DDS topics for specified duration."""
    print("Discovering DDS topics for {} seconds...".format(duration))
    print("-" * 100)
    
    # Create participant
    participant = DomainParticipant(0)
    
    # Create reader for builtin publication data
    reader = DataReader(participant, BuiltinTopicDcpsPublication)
    
    discovered_topics = {}
    start_time = time.time()
    
    while time.time() - start_time < duration:
        # Read all available samples
        samples = reader.take(N=100)
        
        for sample in samples:
            if sample is not None:
                topic_name = sample.topic_name
                type_name = sample.type_name
                
                # Filter for Go2 robot topics
                if topic_name.startswith('rt/'):
                    if topic_name not in discovered_topics:
                        discovered_topics[topic_name] = type_name
                        print(f"Found: {topic_name:40s} -> Type: {type_name}")
        
        time.sleep(0.1)
    
    print("-" * 100)
    print(f"\nTotal discovered: {len(discovered_topics)} topics")
    
    # Print topics of interest
    print("\n" + "=" * 100)
    print("TOPICS OF INTEREST:")
    print("=" * 100)
    
    topics_of_interest = [
        'rt/lf/lowstate',
        'rt/lowstate', 
        'rt/lf/sportmodestate',
        'rt/sportmodestate'
    ]
    
    for topic in topics_of_interest:
        if topic in discovered_topics:
            print(f"✓ {topic:40s} -> Type: {discovered_topics[topic]}")
        else:
            print(f"✗ {topic:40s} -> NOT FOUND")
    
    return discovered_topics

if __name__ == "__main__":
    print("DDS Topic Discovery Tool")
    print("=" * 100)
    discovered = discover_topics(duration=10)
    
    print("\n" + "=" * 100)
    print("SUMMARY - Update dds_bridge.py with these typenames:")
    print("=" * 100)
    
    if 'rt/lf/lowstate' in discovered:
        print(f"LowState_DDS typename should be: '{discovered['rt/lf/lowstate']}'")
    
    if 'rt/lf/sportmodestate' in discovered:
        print(f"SportModeState_DDS typename should be: '{discovered['rt/lf/sportmodestate']}'")
    elif 'rt/sportmodestate' in discovered:
        print(f"SportModeState_DDS typename should be: '{discovered['rt/sportmodestate']}'")
        print("Note: Topic is 'rt/sportmodestate' (without lf/)")
