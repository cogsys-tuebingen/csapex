#!/usr/bin/env python

"""
This scripts allows training samples to be edited
Start with a preview image of a training sample as parameter
Tip: Install ubuntu-tweak and set this script as default for png images
     -> open preview png's of samples to edit them
"""

import os
import sys
import cv
import cv2
import numpy
import yaml
import math
import getpass
import subprocess

# the brush size
radius = 10

change = True
h = 0
w = 0

ux = 0
uy = 0
best_reference = ""

yaw = 0

another = 0

def add(x, y):
    circle(x, y, 255)

def remove(x, y):
    circle(x, y, 0)

def circle(x, y, color):
    global change
    cv2.circle(mask, (x, y), radius, (color, color, color), -1)
    change = True

def crop():
    global img, mask, change, w, h
    
    h, w = img.shape[:2]
    
    x1,y1,x2,y2 = bounds(mask)
    
    img = img[y1:y2, x1:x2]
    mask = mask[y1:y2, x1:x2]

    change = True

def bounds(mask):
    h, w = mask.shape[:2]
    
    minx = w
    maxx = 0
    miny = h
    maxy = 0    
    
    for x in range(w):
        for y in range(h):
            if mask[y,x] > 0:
                minx = min(x, minx);
                miny = min(y, miny);
                maxx = max(x, maxx);
                maxy = max(y, maxy);
            
    return minx,miny, maxx+1,maxy+1;

def callback(e, x, y, mouse, data):
    global ux, uy
    ux = x % w
    uy = y % h
    if mouse == 1:
        add(ux, uy)
    elif mouse == 2:
        remove(ux, uy)

def combine():
    global change, preview, img, mask, combined, w, h, yaw, reference_img, reference_img_crop
    
    combined = numpy.zeros(img.shape,dtype='uint8')
    cv2.bitwise_and(img, img, combined, mask)
    
    # visualize orientation
    angle = yaw - numpy.pi / 2
    r = min(w, h) / 3.0
    f = (w/2, h/2)
    t = (int(f[0] - numpy.cos(angle) * r), int(f[1] + numpy.sin(angle) * r))
    cv2.line(combined, f, t, (0, 0, 255), 3)
    
    # crop reference image
    reference_img_crop = numpy.zeros(img.shape,dtype='uint8')
    rh, rw = reference_img.shape[:2]
    
    scale = 1.0
    if rh > h or rw > w:
        scale = min(h / float(rh), w / float(rw))
    
    sw = int(rw * scale)
    sh = int(rh * scale)
    
    resized = cv2.resize(reference_img, (sw, sh))    

    print reference_img_crop.shape, resized.shape
    
    reference_img_crop[0:sh, 0:sw] = resized
    
    change = False

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

def save():
    global yaw, dist
    
    combine()
    print "save to", preview
    cv2.imwrite(datadir + "img.ppm", img)
    cv2.imwrite(datadir + "mask.ppm", mask)
    cv2.imwrite(preview, combined)

    q = [0,0,0,0]    
    q[0] = float(numpy.cos(yaw/2));
    q[3] = float(numpy.sin(yaw/2));
    
    normalize(q)

    f = open(datadir + 'info.yaml', 'w')
    yaml.dump_all([q[0], q[1], q[2], q[3], float(yaw), float(dist)], f, explicit_start=True)
    f.close();
    
    
    yaw_tmp = 2 * numpy.arccos(q[0]);
    print yaw, yaw_tmp, q[0]
    
def rotate(delta):
    global yaw, change
    yaw += delta
    while yaw > numpy.pi:
        yaw -= 2 * numpy.pi
    while yaw < -numpy.pi:
        yaw += 2 * numpy.pi
    find_closest_ref()   
    change = True
    
def find_closest_ref():
    global reference_files, references, reference_img, best_reference, yaw
    
    best_dist = 100.0
    best = ""
    best_yaw = -1.0
    
    for r in reference_files:
        tmpyaw = float(r[0:r.index("_")])
        diff = abs(yaw - tmpyaw)
        if diff < best_dist:
            best_yaw = tmpyaw
            best_dist = diff
            best = r 
            
    print "yaw =", yaw, "best =", best_yaw
            
    if best != best_reference:
        best_reference = best
    
        reference_img = cv2.imread(references + "/" + best_reference)
        
def gonext():
    global another
    crop()
    save()
    another = 1
    
def goprev():
    global another
    crop()
    save()
    another = -1

def editor(editfile):
    global change, preview, img, mask, combined, datadir, h, w
    global references, reference_files, yaw, dist, best_reference, reference_img, reference_img_crop
    global another        
    
    preview = editfile   
    datadir = preview[:-4] + "/"
    
    if not os.path.exists(datadir):
        return False

    cv.NamedWindow("editor");

    img = cv2.imread(datadir + "img.ppm")
    mask = cv2.imread(datadir + "mask.ppm", 0)
    combined = cv2.bitwise_and(img, img, mask=mask)
    
    h, w = img.shape[:2]
    
    f = open(datadir + 'info.yaml')
    data = yaml.load_all(f)
    raw = [0 for i in range(6)]
    i = 0
    for line in data:
        raw[i] = line
        i += 1
    f.close()

    rot = raw[0:4]
    dist = raw[5]
    yaw = raw[4]
    
    key = "RABOT"
    if key in os.environ:
        rabot = os.environ["RABOT"]
    else:
        rabot = "/home/"+ getpass.getuser() + "/ws/rabot/"
    references = rabot + "/Config/RobotDetection/reference"
    
    reference_files = os.listdir(references)
    
    find_closest_ref()

    cv.SetMouseCallback("editor", callback)
    
    running = True

    while running:   
        if change:
            combine()

        mask3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)    
    
        h, w = img.shape[:2]
        merged = numpy.zeros((2*h, 2*w, 3), numpy.uint8)
        merged[:h, :w] = img
        merged[h:h+h, :w] = mask3
        merged[:h, w:w+w] = combined
        merged[h:h+h, w:w+w] = reference_img_crop
        
        cv2.circle(merged, (ux, uy), radius, (255, 0, 0), 1)
        cv2.circle(merged, (ux+w, uy), radius, (255, 0, 0), 1)
        cv2.circle(merged, (ux, uy+h), radius, (255, 0, 0), 1)
        cv2.circle(merged, (ux+w, uy+h), radius, (255, 0, 0), 1)

        cv2.putText(merged, "distance: " + float.__str__(float(dist)), (15, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (0, 0, 255));
    
        cv2.imshow("editor", merged)
#        cv2.imshow("best match", reference_img)
        key = cv2.waitKey(33)
        
        if key == 27:
            running = False
        elif key == ord('s') or key == ord('S'):
            save()
        elif key == ord('c'):
            crop()
        elif key == ord('+'):
            rotate(0.1)
        elif key == ord('-'):
            rotate(-0.1)
        elif key == 65361: #left
            goprev()
            running = False
        elif key == 65363: #right
            gonext()
            running = False
        elif key == ord('\n'):
            crop()
            save()
            running = False
        elif key != -1:
            print key, "is not bound"
                 

    if another != 0:
        d = editfile[0 : editfile.rfind("/")]
        prefix = d[0 : d.rfind("/")]
        current = editfile[editfile.rfind("/")+1:]
                
        files = os.listdir(d)
        
        # filter out pngs        
        def f(x): return x.find(".png") > 0
        files = filter(f, files)
        
        # sort by name
        list.sort(files)      
       
        # find current
        idx = files.index(current)        
        
        n = len(files)        
        nextone = (idx + another + n) % n
                
        change = True
        another = 0
        return editor(d + "/" + files[nextone])  
                 
    return True

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise RuntimeError("usage: " + sys.argv[0] + " <preview-frame>.png")
    if not editor(sys.argv[1]):
        subprocess.call(['eog', sys.argv[1]])