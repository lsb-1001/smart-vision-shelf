"""Local OpenCV demo/inspection window. No ROS or robot needed in --demo mode."""
import argparse
from pathlib import Path
import time

from shelf_core import Registry, ArrivalVerifier


def dictionary(cv2, name):
    if not hasattr(cv2.aruco,name):
        raise ValueError('Unknown ArUco dictionary: '+name)
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,name))


def detect(cv2, gray, book):
    if hasattr(cv2.aruco,'ArucoDetector'):
        return cv2.aruco.ArucoDetector(book).detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray,book)


def marker_image(cv2, book, marker_id, pixels):
    if hasattr(cv2.aruco,'generateImageMarker'):
        return cv2.aruco.generateImageMarker(book,marker_id,pixels)
    return cv2.aruco.drawMarker(book,marker_id,pixels)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo',action='store_true',help='Generated marker images, actual OpenCV detection')
    parser.add_argument('--image',help='Inspect a saved image; frames are replayed, not live verification')
    parser.add_argument('--registry',default=str(Path(__file__).with_name('locations.json')))
    parser.add_argument('--generate-markers',action='store_true')
    args = parser.parse_args()
    import cv2
    import numpy as np
    registry = Registry.load(args.registry)
    book = dictionary(cv2,registry.data['dictionary'])
    if args.generate_markers:
        folder = Path(__file__).with_name('markers')
        folder.mkdir(exist_ok=True)
        for entry in registry.by_id.values():
            image = marker_image(cv2,book,entry['marker_id'],400)
            image = cv2.copyMakeBorder(image,60,60,60,60,cv2.BORDER_CONSTANT,value=255)
            if not cv2.imwrite(str(folder/f"id_{entry['marker_id']}_{entry['key']}.png"),image):
                raise RuntimeError('Could not save marker image')
        print('Markers:',folder)
        return
    if not args.demo and not args.image:
        parser.error('Use --demo, --image FILE or --generate-markers')
    source = cv2.imread(args.image) if args.image else None
    if args.image and source is None:
        parser.error('Could not read image')
    verifier = ArrivalVerifier(registry)
    target, visible, camera_on, sequence = 'B', [2], True, 0
    print('Keys: a/b/c/d target; 1/2/3/4 visible marker; n none; u unknown; m multiple;')
    print('v simulate NAV ARRIVED and begin verification; x simulate camera loss; q quit')
    try:
        while True:
            now = time.monotonic()
            if args.demo:
                image = np.full((480,640,3),255,np.uint8)
                if camera_on:
                    size = 180 if len(visible)>1 else 240
                    for i, marker in enumerate(visible):
                        patch = marker_image(cv2,book,marker,size)
                        y,x = 160, 80+i*260 if len(visible)>1 else 200
                        image[y:y+size,x:x+size] = cv2.cvtColor(patch,cv2.COLOR_GRAY2BGR)
                else:
                    image[:] = 60
            else:
                image = source.copy()
            if camera_on:
                corners,ids,_ = detect(cv2,cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),book)
                found = [] if ids is None else [int(v) for v in ids.flatten()]
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(image,corners,ids)
                # Saved image replay never confirms arrival.
                if args.image:
                    verifier.ids = found
                else:
                    verifier.observe(found,now,fresh=True)
            result = verifier.tick(now)
            banner = [
                'DEMO: synthetic markers + simulated navigation' if args.demo else 'IMAGE INSPECTION ONLY',
                f'Target: {target} / ID {registry.get(target)["marker_id"]}',
                f'Observed: {result["observed_ids"]}  State: {result["state"]}',
                'a/b/c/d target | 1-4 marker | v verify | n/u/m | x camera | q quit',
                'Coordinates: UNREGISTERED | Identity check only']
            panel = np.full((155,640,3),35,np.uint8)
            for i,line in enumerate(banner):
                cv2.putText(panel,line,(8,23+i*28),cv2.FONT_HERSHEY_SIMPLEX,0.46,(0,255,255),1)
            cv2.imshow('Shelf identity verification',np.vstack([panel,image]))
            key = cv2.waitKey(30) & 0xff
            if key in [ord('q'),27]:
                break
            if key in map(ord,'abcd'):
                verifier.cancel()
                target = dict(a='A',b='B',c='C',d='DELIVERY')[chr(key)]
            elif key in map(ord,'1234'):
                visible,camera_on = [int(chr(key))],True
            elif key == ord('n'):
                visible,camera_on = [],True
            elif key == ord('u'):
                visible,camera_on = [49],True
            elif key == ord('m'):
                visible,camera_on = [1,2],True
            elif key == ord('x'):
                camera_on = not camera_on
            elif key == ord('v') and args.demo:
                verifier.cancel()
                sequence += 1
                verifier.begin(target,f'demo-{sequence}',time.monotonic(),navigation_arrived=True)
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
