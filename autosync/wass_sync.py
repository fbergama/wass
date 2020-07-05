import os
import subprocess
from glob import glob
import cv2
import numpy as np

pathname = os.environ['HOME'] + '/WASS_RUN/sync/'
last_frame = 20000 # last frame number from synchronized videos
os.chdir(pathname)
clip_list = glob('*.mp4')
ref_clip_index = 0 # first clip used as reference
ref_clip = clip_list[ref_clip_index]
clip_list.pop(ref_clip_index) #remove the reference clip from the list
command = "ffmpeg -i {} -vn -acodec pcm_s16le -ar 44100 -ac 2 {}".format(ref_clip,"ref.wav")
os.system(command)
results = []
results.append((ref_clip, 0)) #the reference clip has an offset of 0
for clip in clip_list:
    clipfile = clip.split(".")[0] + ".wav"
    command = "ffmpeg -i {0} -vn -acodec pcm_s16le -ar 44100 -ac 2 {1}".format(clip,clipfile)
    os.system(command)
    command = "praat crosscorrelate.praat ref.wav {}".format(clipfile)
    result = subprocess.check_output(command, shell=True)
    results.append((clip, result.decode("utf-8").split("\n")[0]))

for result in results:
    clip_start = 0
    clip_dur = 3000   #in seconds
    in_name = result[0]
    out_name = in_name.split('.')[0] + "_sync.mp4"
    offset = round(float(result[1]),3)
    print('offset:'+str(offset))
    clip_start += offset
    command = "ffmpeg -i {0} -c:a copy -c:v libx264 -crf 18 -ss {1} -to {2} {3}".format(in_name,str(clip_start),str(clip_dur),out_name)
    os.system(command)
command = "ffmpeg -i cam0_sync.mp4 -c:a copy -c:v libx264 -crf 18 -vsync 2 -r 10 cam0_fps10.mp4"
command2 = "ffmpeg -i cam1_sync.mp4 -c:a copy -c:v libx264 -crf 18 -vsync 2 -r 10 -vf transpose=2,transpose=2 cam1_fps10.mp4"
os.system(command)
os.system(command2)
paths_id = '01'
for i in paths_id:
    pathname_fig    = pathname + 'cam'+i+'/'
    filename        = 'cam'+i+'_fps10.mp4'
    cap = cv2.VideoCapture(pathname + filename)
    ret, frame = cap.read()
    count = 0
    ret = True
    while ret:
        cv2.imwrite(pathname_fig + "%06d.tif" % count, frame)
        ret, frame = cap.read()
        print ('make a new frame:', count, ret)
        count += 1
        if count + 1 == last_frame:
           break
