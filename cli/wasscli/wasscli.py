"""
wasscli
Copyright (C) 2022 Filippo Bergamasco

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import shutil
import colorama
import subprocess
import os
import numpy as np
import glob
import tqdm
from PyInquirer import prompt, Separator, Validator, ValidationError
#from wassgridsurface import wassgridsurface_main

colorama.init()

VERSION = "0.1.2"


WASS_PIPELINE = {
    "wass_prepare": None,
    "wass_match": None,
    "wass_autocalibrate": None,
    "wass_stereo": None
}
SUPPORTED_IMAGE_FORMATS = ["tif", "tiff", "png", "jpg", "jpeg"]



#--------------------------------------------------------------------------------
# Helper functions
#--------------------------------------------------------------------------------

def find_wass_pipeline():
    print("Searching for WASS pipeline executables...", end="")
    for program_name in WASS_PIPELINE.keys():
        program_exe = shutil.which( program_name )
        if program_exe is None:
            print("")
            print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
            print( "%s not found. Please add the WASS executable files to system PATH"%program_name )
            return False

        ret = subprocess.run(program_exe, capture_output=False, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL )
        if ret.returncode != 0:
            print("")
            print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
            print( "%s found but returns an invalid status code. Please update WASS and install it again to fix."%program_name )
            return False


        WASS_PIPELINE[ program_name ] = program_exe

    print( colorama.Fore.GREEN+"OK"+colorama.Style.RESET_ALL)
    return True



def check_workdir_structure():
    required_dirs = ["config","input","input/cam0","input/cam1"]

    for d in required_dirs:
        if not os.path.exists(d):
            return False

    return True


def initialize_working_directory():

    if not os.path.exists("./config"):
        os.mkdir( "config")
        ret = subprocess.run( [WASS_PIPELINE["wass_match"], "--genconfig"], capture_output=False, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL )
        shutil.move("matcher_config.txt", "config/matcher_config.txt")
        ret = subprocess.run( [WASS_PIPELINE["wass_stereo"], "--genconfig"], capture_output=False, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL )
        shutil.move("stereo_config.txt", "config/stereo_config.txt")

    if not os.path.exists("input"):
        os.mkdir("input")
    if not os.path.exists("input/cam0"):
        os.mkdir("input/cam0")
    if not os.path.exists("input/cam1"):
        os.mkdir("input/cam1")
    if not os.path.exists("output"):
        os.mkdir("output")

    print("")
    print("WASS working directory initialized. Please:")
    print(" 1) copy your calibration data intrinsics_00.xml, intrinsics_01.xml, distortion_00.xml and distortion_01.xml in the config directory.")
    print(" 2) copy your images into input/cam0 and input/cam1 directories")
    print("")



def get_image_files( directory ):

    for extension in SUPPORTED_IMAGE_FORMATS:
        imgfiles = glob.glob( "%s/*.%s"%(directory,extension))
        if len(imgfiles)>1:
            return sorted(imgfiles)
    return []



def get_workdirs():
    workdirs = sorted(glob.glob("output/*_wd"))
    if len(workdirs) == 0:
        print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
        print("output/ directory looks empty. Please run Prepare first.")
        return None
    return workdirs


#--------------------------------------------------------------------------------
# WASS Pipeline operations
#--------------------------------------------------------------------------------

def do_prepare():

    if len(os.listdir("output/")) != 0:
        print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
        print("output/ directory must be empty to continue. Please manually remove all the content before attempting to prepare again.")
        return False


    calib_files = ["config/intrinsics_00.xml","config/intrinsics_01.xml","config/distortion_00.xml","config/distortion_01.xml"]
    print("Checking calibration files...")
    for calib_file in calib_files:
        if not os.path.exists( calib_file ):
            print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
            print( "%s not found, aborting"%calib_file )
            return False

    print("Checking input/cam0 and input/cam1...")
    cam0_files = get_image_files("input/cam0")
    cam1_files = get_image_files("input/cam1")

    if len(cam0_files) == 0:
        print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
        print("No image found in input/cam0 with the following formats: ", SUPPORTED_IMAGE_FORMATS)
        return False

    if len(cam1_files) == 0:
        print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
        print("No image found in input/cam1 with the following formats: ", SUPPORTED_IMAGE_FORMATS)
        return False

    if len(cam0_files) != len(cam1_files):
        print( colorama.Fore.RED+"ERROR: "+colorama.Style.RESET_ALL, end="")
        print("cam0 and cam1 directories contain a different set of images. Aborting")
        return False

    N = len(cam0_files)
    print(colorama.Fore.GREEN+("%d"%N)+colorama.Style.RESET_ALL+" stereo pairs found!")


    class NumFramesValidator(Validator):
        def validate(self, document):
            if int(document.text)<3 or int(document.text)>N:
                raise ValidationError(
                    message='Please enter a valid number',
                    cursor_position=len(document.text))  # Move cursor to end
    questions = [
        {
            'type': 'input',
            'name': 'framestoprepare',
            'message': 'How many stereo frames do you want to prepare? (3 ... %d)'%N,
            'validate': NumFramesValidator
        }
    ]
    answers = prompt(questions)

    print("Running wass_prepare... please be patient")
    for t in tqdm.trange(int(answers["framestoprepare"])):
        wdirname = "output/%06d_wd"%t
        ret = subprocess.run( [WASS_PIPELINE["wass_prepare"],"--workdir", wdirname, "--calibdir", "config/", "--c0", cam0_files[t], "--c1", cam1_files[t]], capture_output=True )
        if ret.returncode != 0:
            print( colorama.Fore.RED+("ERROR while running wass_prepare on frame %06d ****************"%t)+colorama.Style.RESET_ALL)
            print(ret.stdout.decode("ascii"))
            print( colorama.Fore.RED+("*********************************************************************")+colorama.Style.RESET_ALL)
            return False

    print( colorama.Fore.GREEN+("Prepare completed!")+colorama.Style.RESET_ALL)
    return True



def do_match():
    workdirs = get_workdirs()
    if workdirs is None: return False

    suggested_num_to_match = min( 50, len(workdirs))

    class NumFramesValidator(Validator):
        def validate(self, document):
            if int(document.text)<0 or int(document.text)>len(workdirs):
                raise ValidationError(
                    message='Please enter a valid number',
                    cursor_position=len(document.text))  # Move cursor to end
    questions = [
        {
            'type': 'input',
            'name': 'framestomatch',
            'message': 'How many frames do you want to use for matching? (1 ... %d, suggested: %d)'%(len(workdirs), suggested_num_to_match),
            'validate': NumFramesValidator
        }
    ]
    answers = prompt(questions)

    indices = np.random.permutation( len(workdirs) )[ :int(answers["framestomatch"])]
    print("Matcher will use the following frames: ", indices)

    print("Running wass_match... please be patient")
    for t in tqdm.tqdm( indices ):
        wdirname = "output/%06d_wd"%t
        ret = subprocess.run( [WASS_PIPELINE["wass_match"], "config/matcher_config.txt", wdirname], capture_output=True )
        if ret.returncode != 0:
            print( colorama.Fore.RED+("ERROR while running wass_match on frame %06d ****************"%t)+colorama.Style.RESET_ALL)
            print(ret.stdout.decode("ascii"))
            print( colorama.Fore.RED+("*********************************************************************")+colorama.Style.RESET_ALL)
            return False
        else:
            tqdm.tqdm.write(ret.stdout.decode("ascii"))

    print( colorama.Fore.GREEN+("Match completed!")+colorama.Style.RESET_ALL)
    return True


def do_autocalibrate():
    workdirs = get_workdirs()
    if workdirs is None: return False

    with open('output/workspaces.txt', 'w') as f:
        f.write('\n'.join(workdirs))

    print("Running wass_autocalibrate... please be patient")
    ret = subprocess.run( [WASS_PIPELINE["wass_autocalibrate"], "output/workspaces.txt"], capture_output=True )
    if ret.returncode != 0:
        print( colorama.Fore.RED+("ERROR while running wass_autocalibrate ****************")+colorama.Style.RESET_ALL)
        print(ret.stdout.decode("ascii"))
        print( colorama.Fore.RED+("*******************************************************")+colorama.Style.RESET_ALL)
        return False
    else:
        tqdm.tqdm.write(ret.stdout.decode("ascii"))
        print( colorama.Fore.GREEN+("Autocalibrate completed!")+colorama.Style.RESET_ALL)

    return True


def do_stereo():
    workdirs = get_workdirs()
    if workdirs is None: return False
    questions = [
        {
            'type': 'confirm',
            'name': 'processall',
            'message': 'Do you want to reconstruct all the sequence? Select N to reconstruct 000000_wd only.',
            'default': True
        }
    ]
    answers = prompt(questions)

    with open("output/planes.txt", "w") as fplanes:

        while( True ):
            print("Running wass_stereo... please be patient")
            for t in tqdm.trange( len(workdirs) if answers["processall"] else 1, unit="frames" ):
                wdirname = "output/%06d_wd"%t
                ret = subprocess.run( [WASS_PIPELINE["wass_stereo"],"config/stereo_config.txt", wdirname ], capture_output=True )
                if ret.returncode != 0:
                    print( colorama.Fore.RED+("ERROR while running wass_stereo on frame %06d ****************"%t)+colorama.Style.RESET_ALL)
                    print(ret.stdout.decode("ascii"))
                    print( colorama.Fore.RED+("*********************************************************************")+colorama.Style.RESET_ALL)
                    return False
                else:
                    tqdm.tqdm.write(ret.stdout.decode("ascii"))
                    with open( wdirname+"/plane.txt", "r" ) as finplane:
                        s = finplane.readlines()
                        fplanes.write((" ".join(s).replace("\n",""))+"\n")

            print( colorama.Fore.GREEN+("Stereo completed!")+colorama.Style.RESET_ALL)

            if not answers["processall"]:
                print("Check output/000000_wd and edit config/stereo_config.txt if needed.")
                questions = [
                    {
                        'type': 'confirm',
                        'name': 'tryagain',
                        'message': 'Try again?',
                        'default': False
                    }
                ]
                answers2 = prompt(questions)
                if not answers2["tryagain"]:
                    return True
            else:
                return True

#--------------------------------------------------------------------------------
# Gridding operations
#--------------------------------------------------------------------------------
# TODO: To be implemented

def do_grid():
    pass


#--------------------------------------------------------------------------------
# Main
#--------------------------------------------------------------------------------


def wasscli_main():
    print("\n WASS-cli v.", VERSION )
    print("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\nCopyright (C) Filippo Bergamasco 2022 \n\n")

    if not find_wass_pipeline():
        return -1

    #os.chdir("/Users/fibe/tmp/aaa")

    print("Current directory is: ", os.getcwd() )
    if not check_workdir_structure():
        print("Current directory does not appear as a WASS working directory.")
        questions = [
            {
                'type': 'confirm',
                'name': 'initialize',
                'message': 'Current directory is not a WASS working directory. Do you want to initialize it?',
                'default': True
            }
        ]
        answers = prompt(questions)
        if answers["initialize"]:
            initialize_working_directory()
            return 0

    while True:
        questions = [
            {
                "type": "list",
                "name": "wass_step",
                "message": "What do you want to do?",
                "choices": [
                    "Prepare",
                    "Match",
                    "Autocalibrate",
                    "Stereo",
                    # Separator(),
                    # "Grid",
                    # "Plot",
                    Separator(),
                    "Quit"
                ]
            }
        ]
        answers = prompt(questions)
        if answers["wass_step"] == "Prepare":
            do_prepare()
        elif answers["wass_step"] == "Match":
            do_match()
        elif answers["wass_step"] == "Autocalibrate":
            do_autocalibrate()
        elif answers["wass_step"] == "Stereo":
            do_stereo()
        elif answers["wass_step"] == "Quit":
            print("Bye.")
            return 0
        else:
            print("Choice not available")



if __name__ == "__main__":
    print("REMOVE THIS!!!!")
    wasscli_main()
