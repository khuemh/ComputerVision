import cv2 as cv 
import numpy as np
import os


class basicIO():
    def np_array():
        img = np.zeros((3, 3), dtype = np.uint8)
        img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        print(img)
        print(img.shape)


    def img_RW():
        img_gray = cv.imread("img_01.jpg", cv.IMREAD_GRAYSCALE)
        cv.imwrite("img_01_gray.jpg", img_gray)


    def rawbytes():
        randomArray = bytearray(os.urandom(120000))
        numArray = np.array(randomArray)
        
        grayImg = numArray.reshape(300, 400)
        cv.imwrite("randomGray.jpg", grayImg)

        bgrImg = numArray.reshape(100, 400, 3)
        cv.imwrite("randomBGR.jpg", bgrImg)


    def imgManipulate():
        cv.namedWindow('Window')
        img = cv.imread('/home/khuehm/wrkspc/ComputerVision/pycv/chap2/black.png')
        img[0, 0] = [255, 255, 255]
        cv.imshow('Window', img)
        cv.waitKey(0)


    def imgItem():
        img = cv.imread('/home/khuehm/wrkspc/ComputerVision/pycv/chap2/black.png')
        print(img.item(150, 120, 0))
        img.itemset((150, 120, 0), 255) 
        print(img.item(150, 120, 0))

    def imgROI():
        img = cv.imread('/home/khuehm/wrkspc/ComputerVision/pycv/chap2/img_01.jpg')
        my_roi = img[0:100, 0:100]
        img[300:400, 300:400] = my_roi
        cv.imwrite('img_roi.jpg', img)


    def video_RW():
        vCapture = cv.VideoCapture('/home/khuehm/wrkspc/ComputerVision/pycv/resources/sample.avi')
        fps = vCapture.get(cv.CAP_PROP_FPS)
        size = (int(vCapture.get(cv.CAP_PROP_FRAME_WIDTH)), int(vCapture.get(cv.CAP_PROP_FRAME_HEIGHT)))
        vWriter = cv.VideoWriter('outputSample.avi', cv.VideoWriter_fourcc('I', '4', '2', '0'), fps, size)

        success, frame = vCapture.read()
        while success:
            vWriter.write(frame)
            success, frame = vCapture.read()


    def stream_RW():
        vCapture = cv.VideoCapture(0)
        fps = 30
        size = (int(vCapture.get(cv.CAP_PROP_FRAME_WIDTH)), int(vCapture.get(cv.CAP_PROP_FRAME_HEIGHT)))
        vWriter = cv.VideoWriter('streamOuput.avi', cv.VideoWriter_fourcc('I', '4', '2', '0'), fps, size)

        success, frame = vCapture.read()
        numFrames = 5 * fps - 1
        while numFrames > 0 and success:
            vWriter.write(frame)
            success, frame = vCapture.read()
            numFrames -= 1

        print('Done!')
        vCapture.release()

    
    def displayImage():
        img = cv.imread('/home/khuehm/wrkspc/ComputerVision/pycv/resources/img_01.jpg')
        cv.imshow('Display Image', img)
        cv.waitKey(0)
        cv.destroyAllWindows()



if __name__ == '__main__':
    #basicIO.np_array()
    #basicIO.img_RW()
    #basicIO.rawbytes()
    #basicIO.imgManipulate()
    #basicIO.imgItem()
    #basicIO.imgROI()
    #basicIO.video_RW()
    #basicIO.stream_RW()
    basicIO.displayImage()