import cv2
import numpy

im = cv2.imread('leaves1.png')

print(type(im))
print(im.dtype)
print(im.shape)
cv2.imshow('im', im)
cv2.waitKey(0)
cv2.destroyAllWindows()
