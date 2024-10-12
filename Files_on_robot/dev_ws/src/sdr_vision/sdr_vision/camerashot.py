import cv2 
import os

###
## additional tool for local testing
## take screenshot from camera
###

def main(args=None):
    vid = cv2.VideoCapture(0) 
    
    counter = 0

    while(True): 
        ret, frame = vid.read() 

        if cv2.waitKey(1) == ord('w'):
            cv2.imwrite("screenshot_images/" + "sherry_" + str(counter) + ".jpg", frame) 
            counter += 1
            print ("write image " + str(counter))
            cwd = os.getcwd()
            print(cwd)

        cv2.imshow("camera", frame)

    vid.release() 
    cv2.destroyAllWindows() 

if __name__ == '__main__':
    main()