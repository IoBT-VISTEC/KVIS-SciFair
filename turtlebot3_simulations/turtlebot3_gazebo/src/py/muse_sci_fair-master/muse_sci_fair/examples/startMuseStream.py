"""
Starting a Stream

This example shows how to search for available Muses and
create a new stream
"""
from muselsl import stream, list_muses

if __name__ == "__main__":

    muses = list_muses()
 
    stream('00:55:da:b3:d8:4c')


    if not muses:
        print('No Muses found')
    else:
        print("found muse ", muses[0]['address'])
        stream(muses[0]['address'])

        # Note: Streaming is synchronous, so code here will not execute until the stream has been closed
        print('Stream has ended')
