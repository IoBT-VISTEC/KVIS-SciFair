"""
Record from a Stream

This example shows how to record data from an existing Muse LSL stream
"""
from muselsl import record, viewer_v2
from threading import Thread
muse_name1 = 'DC11'
muse_name2 = 'BACD'
def worker():
    print("Running View with " + muse_name1 + '\n')
    viewer_v2.view(muse_name1)

def worker2():
    print("Running View with " + muse_name2 + '\n')
    viewer_v2.view(muse_name2)

if __name__ == "__main__":
    muse_name1 = 'DC11'
    muse_name2 = 'BACD'
    muse_name3 = 'D84C'
    t1 = Thread(target=worker)
    t2 = Thread(target=worker2)
    t1.setDaemon(True)
    t2.setDaemon(True)
    t1.start()
    t2.start()
    print('Recording has ended')
    while True:
        pass
