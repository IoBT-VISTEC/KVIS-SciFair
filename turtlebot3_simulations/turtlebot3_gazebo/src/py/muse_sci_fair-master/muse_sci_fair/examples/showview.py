"""
Record from a Stream

This example shows how to record data from an existing Muse LSL stream
"""
from muselsl import record, viewer_v2

if __name__ == "__main__":

    # Note: an existing Muse LSL stream is required
    # record(5)
    viewer_v2.view('C00E')
    # viewer_v2.view('BACD')
    # viewer_v2.view('C08D')
    # Note: Recording is synchronous, so code here will not execute until the stream has been closed
    print('Recording has ended')
