import cv2 as cv
import os

def draw_yolo_bboxes(image_path, label_path):
    img = cv.imread(image_path)
    if img is None:
        print("Imagem não encontrada:", image_path)
        return

    h_img, w_img = img.shape[:2]

    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue

            cls, x, y, w, h = map(float, parts)
            # YOLO usa coordenadas normalizadas, então convertemos:
            x_center = x * w_img
            y_center = y * h_img
            width = w * w_img
            height = h * h_img

            # Obtem coordenadas do retângulo
            x1 = int(x_center - width / 2)
            y1 = int(y_center - height / 2)
            x2 = int(x_center + width / 2)
            y2 = int(y_center + height / 2)

            # Desenha bounding box
            cv.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(img, str(int(cls)), (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv.imshow("Bounding Boxes", img)
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))

    img_path = os.path.join(current_dir, "0001_rotated.png")
    label_path = os.path.join(current_dir, "0001_rotated.txt")

    draw_yolo_bboxes(img_path, label_path)
