#importing libraries
import cv2
import apriltag


def detector(gray,len_prev_res):
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    if len_prev_res != len(results):
     print("[INFO] {} total AprilTags detected".format(len(results)))
    return results

def detection_res(frame,results):
    image = frame
    for r in results:
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)

        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
       #print("[INFO] tag family: {}".format(tagFamily))

    return image

def main():
    print("[INFO] loading video...")
    capture = cv2.VideoCapture(-1)
    print("[INFO] detecting AprilTags...")
    len_prev_res = 0

    while True:
     isTrue , frame = capture.read()
     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     results = detector(gray,len_prev_res)
     len_prev_res = len(results)
     image = detection_res(frame,results)
     cv2.imshow('Video',image)

     if cv2.waitKey(20) & 0xFF==ord('d'):
      break

    capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
