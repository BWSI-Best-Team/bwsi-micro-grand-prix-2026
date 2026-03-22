import sys

sys.path.insert(0, '../../library')
import racecar_core
rc = racecar_core.create_racecar()

def start():
    pass 

 
def update():
    pass 


def update_slow():
    pass 


if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()

