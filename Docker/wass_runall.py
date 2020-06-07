import requests
from tqdm import tqdm
import time

ADDR = 'http://wass:8080'
last_value = 0
last_max = 0

def refresh_status( pbar ):

    global last_max
    global last_value
    r = requests.get(ADDR+'/fullstatus').json()

    curr_max = int(r["progress_max"])
    curr_value = r["progress"]

    if (curr_max != last_max) or (curr_value<last_value):
        last_value = 0
        last_max = curr_max
        pbar.reset()
        pbar.total = curr_max


    pbar.set_description("WASS status: %s"%r["current_status"] )

    pbar.update( curr_value-last_value )
    last_value = curr_value

    return r["current_status"]=="idle", r["completed_tasks"]


def run_next_task( completedtasks ):

    if not completedtasks["dense"]:
        if not completedtasks["matchmerge"]:
            if not completedtasks["match"]:
                if not completedtasks["prepare"]:
                    requests.get(ADDR+"/doprepare")
                    return False

                requests.get(ADDR+"/domatch")
                return False

            requests.get(ADDR+"/domatchmerge")
            return False

        requests.get(ADDR+"/dodense")
        return False

    return True


def do_main():

    pbar = tqdm( total=999, unit="frames" )

    while True:
        time.sleep(2)
        isidle, completedtasks = refresh_status( pbar )
        if isidle:
            if run_next_task( completedtasks ):
                print("All tasks completed.")
                return




if __name__ == "__main__":
    do_main()
