#!/usr/bin/env python2

def resize(obstacle):
    img = obstacle
    #temp_img =img.copy()
    pixel_chg_img = []
    grid_resize = 0
    for k in range(0,399,4): # 0,2,4 so on (*for step)
        mask_average = 0
        resize_img = []
        for j in range(0,399,4):
            mask_average_one = int(img[k][j]) + int(img[k][j+1]) + int(img[k][j+2]) + int(img[k][j+3])
            mask_average_two = int(img[k+1][j]) + int(img[k+1][j+1]) + int(img[k+1][j+2]) + int(img[k+1][j+3])
            mask_average_three = int(img[k+2][j]) + int(img[k+2][j+1]) + int(img[k+2][j+2]) + int(img[k+2][j+3])
            mask_average_four = int(img[k+3][j]) + int(img[k+3][j+1]) + int(img[k+3][j+2]) + int(img[k+3][j+3])
            mask_average = mask_average_one + mask_average_two + mask_average_three + mask_average_four
            if mask_average == 0:
                resize_img.append(0)
            else:
                resize_img.append(255)
        pixel_chg_img.append(resize_img)
        grid_resize = np.asarray(pixel_chg_img, dtype=np.uint8)
    return grid_resize