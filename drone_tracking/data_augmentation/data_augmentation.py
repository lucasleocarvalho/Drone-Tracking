import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import os 

def bright(imgs, labels, current_dir, bright_amounght):
    '''Aumentar o brilho'''

    for name, path in imgs:
        img = cv.cvtColor(cv.imread(path), cv.COLOR_BGR2HSV)
        h, s, v = cv.split(img)

        v[v > 255 - bright_amounght] = 255
        v[v <= 255 - bright_amounght] += bright_amounght

        bright = cv.cvtColor(cv.merge((h, s, v)), cv.COLOR_HSV2BGR)
        img_path = os.path.join(current_dir, name + '_bright.png')
        cv.imwrite(img_path, bright)

    for name, path in labels:
        with open(path, 'r', encoding='utf-8') as f:
            label_data = f.read()
        label_path = os.path.join(current_dir, name + '_bright.txt')
        with open(label_path, 'w', encoding='utf-8') as f:
            f.write(label_data)

def blur(imgs, labels, current_dir, kernell):
    '''Adiciona desfoque'''

    for name, path in imgs:
        img = cv.imread(path)
        blurred = cv.GaussianBlur(img, kernell, 0)
        img_path = os.path.join(current_dir, name + '_blurred.png')
        cv.imwrite(img_path, blurred)

    for name, path in labels:
        with open(path, 'r', encoding='utf-8') as f:
            label_data = f.read()
        label_path = os.path.join(current_dir, name + '_blurred.txt')
        with open(label_path, 'w', encoding='utf-8') as f:
            f.write(label_data)

def noise(imgs, labels, current_dir, noise_amounght):
    '''Aplicar ruido gaussiano'''

    for name, path in imgs:
        img = cv.imread(path)
        noise = img.astype(np.float32) + np.random.normal(0, noise_amounght, img.shape).astype(np.float32)
        img_path = os.path.join(current_dir, name + '_noise.png')
        noise = np.clip(noise, 0, 255).astype(np.uint8)
        cv.imwrite(img_path, noise)

    for name, path in labels:
        with open(path, 'r', encoding='utf-8') as f:
            label_data = f.read()
        label_path = os.path.join(current_dir, name + '_noise.txt')
        with open(label_path, 'w', encoding='utf-8') as f:
            f.write(label_data)

def flip(imgs, labels, current_dir):
    '''Girar 180°'''
    
    for name, path in imgs:
        img = cv.imread(path)
        rotated = cv.rotate(img, cv.ROTATE_180)
        img_path = os.path.join(current_dir, name + '_rotated.png')
        cv.imwrite(img_path, rotated)

    for name, path in labels:
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        rotated_lines = []
        for line in lines:
            cls, x, y, w, h = line.strip(). split()
            x_new = 1.0 - float(x)
            y_new = 1.0 - float(y)

            rotated_line = f"{cls} {x_new:.6f} {y_new:.6f} {w} {h}\n"
            rotated_lines.append(rotated_line)

        label_path = os.path.join(current_dir, name + '_rotated.txt')
        with open(label_path, 'w', encoding='utf-8') as f:
            f.writelines(rotated_lines)

def gray_scale(imgs, labels, current_dir):
    '''Colocar em escala de cinza'''
    
    for name, path in imgs:
        img = cv.imread(path)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_path = os.path.join(current_dir, name + '_gray.png')
        cv.imwrite(img_path, gray)

    for name, path in labels:
        with open(path, 'r', encoding='utf-8') as f:
            label_data = f.read()
        label_path = os.path.join(current_dir, name + '_gray.txt')
        with open(label_path, 'w', encoding='utf-8') as f:
            f.write(label_data)
    

    

def main():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    imgs_and_labels = os.path.join(current_dir, 'imgs_and_labels')
    content = os.listdir(imgs_and_labels)

    imgs = []
    labels = []

    for file in content:
        name, ext = os.path.splitext(file)
        ext = ext.lower()
        full_path = os.path.join(imgs_and_labels, file)

        if ext == '.txt':
            labels.append((name, full_path))
        else:
            imgs.append((name, full_path))
        
    labels.sort(key=lambda x: x[0])
    imgs.sort(key=lambda x: x[0])

    gray_c = True
    noise_c = True
    bright_c = True
    flip_c = True
    blur_c = True

    if gray_c == True:
        gray_scale(imgs, labels, imgs_and_labels)
    if noise_c == True:
        noise(imgs, labels, imgs_and_labels, noise_amounght=25)
    if bright_c == True:
        bright(imgs, labels, imgs_and_labels, bright_amounght=50)
    if flip_c == True:
        flip(imgs, labels, imgs_and_labels)
    if blur_c == True:
        blur(imgs, labels, imgs_and_labels, kernell=(5, 5))

    

if __name__ == "__main__":
    main()