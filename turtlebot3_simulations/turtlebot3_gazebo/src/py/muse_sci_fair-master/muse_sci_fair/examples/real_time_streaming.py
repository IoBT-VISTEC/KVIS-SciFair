"""
Record from a Stream

This example shows how to record data from an existing Muse LSL stream
"""
from muselsl import streaming

if __name__ == "__main__":

    # Note: an existing Muse LSL stream is required
    streaming(5)

    # Note: Recording is synchronous, so code here will not execute until the stream has been closed
    print('Recording has ended')