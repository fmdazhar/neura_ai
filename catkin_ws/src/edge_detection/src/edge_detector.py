#edge_detector.py
import os
import cv2 as cv
import numpy as np

#######################################
# Optional: Custom CropLayer for HED
#######################################
class CropLayer(object):
    def __init__(self, params, blobs):
        self.xstart = 0
        self.xend = 0
        self.ystart = 0
        self.yend = 0

    def getMemoryShapes(self, inputs):
        inputShape, targetShape = inputs[0], inputs[1]
        batchSize, numChannels = inputShape[0], inputShape[1]
        height, width = targetShape[2], targetShape[3]
        self.ystart = (inputShape[2] - targetShape[2]) // 2
        self.xstart = (inputShape[3] - targetShape[3]) // 2
        self.yend = self.ystart + height
        self.xend = self.xstart + width
        return [[batchSize, numChannels, height, width]]

    def forward(self, inputs):
        return [inputs[0][:, :, self.ystart:self.yend, self.xstart:self.xend]]

#######################################
# Main EdgeDetector class
#######################################
class EdgeDetector:

    # Fixed HED model constants
    HED_PROTOTXT   = "/root/catkin_ws/src/edge_detection/data/model/deploy.prototxt"
    HED_CAFFEMODEL = "/root/catkin_ws/src/edge_detection/data/model/hed_pretrained_bsds.caffemodel"
    HED_WIDTH      = 500
    HED_HEIGHT     = 500

    def __init__(self,
                 method="canny",    # "canny", "sobel", "laplacian", "prewitt", "roberts", or "hed"
                 canny_low_threshold=150,
                 canny_high_threshold=250,
                 sobel_ksize=3,
                 laplacian_ksize=3):
        """
        :param method: which edge method to use: "canny", "sobel", "laplacian", "prewitt", "roberts", or "hed"
        :param canny_low_threshold: Canny low threshold
        :param canny_high_threshold: Canny high threshold
        :param sobel_ksize: kernel size for Sobel
        :param laplacian_ksize: kernel size for Laplacian
        """
        self.method = method.lower()
        self.canny_low_threshold = canny_low_threshold
        self.canny_high_threshold = canny_high_threshold
        self.sobel_ksize = sobel_ksize
        self.laplacian_ksize = laplacian_ksize

        # Precompute kernels for Prewitt and Roberts
        self.prewitt_kernel_x = np.array([[1,  0, -1],
                                          [1,  0, -1],
                                          [1,  0, -1]], dtype=np.float32)

        self.prewitt_kernel_y = np.array([[ 1,  1,  1],
                                          [ 0,  0,  0],
                                          [-1, -1, -1]], dtype=np.float32)

        self.roberts_kernel1 = np.array([[1, 0],
                                         [0, -1]], dtype=np.float32)

        self.roberts_kernel2 = np.array([[0,  1],
                                         [-1, 0]], dtype=np.float32)

        # If we're using HED, load the model once
        self.hed_net = None


    def detect_edges(self, img_bgr):
        """
        Main dispatch to the correct method.
        """
        if self.method == "hed":
            return self._detect_edges_hed(img_bgr)
        else:
            return self._detect_edges_classical(img_bgr)

    def _detect_edges_classical(self, img_bgr):
        """
        Handle classical operators: Canny, Sobel, Laplacian, Prewitt, Roberts.
        """
        gray = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
        gray_blurred = cv.GaussianBlur(gray, (5,5), 0)

        if self.method == "canny":
            return cv.Canny(gray_blurred, self.canny_low_threshold, self.canny_high_threshold)
        elif self.method == "sobel":
            sobel_64f = cv.Sobel(gray_blurred, cv.CV_64F, 1, 1, ksize=self.sobel_ksize)
            return cv.convertScaleAbs(sobel_64f)
        elif self.method == "laplacian":
            lap_64f = cv.Laplacian(gray_blurred, cv.CV_64F, ksize=self.laplacian_ksize)
            return cv.convertScaleAbs(lap_64f)
        elif self.method == "prewitt":
            gx = cv.filter2D(gray_blurred, -1, self.prewitt_kernel_x)
            gy = cv.filter2D(gray_blurred, -1, self.prewitt_kernel_y)
            return cv.convertScaleAbs(gx + gy)
        elif self.method == "roberts":
            r1 = cv.filter2D(gray_blurred, -1, self.roberts_kernel1)
            r2 = cv.filter2D(gray_blurred, -1, self.roberts_kernel2)
            return cv.convertScaleAbs(r1 + r2)
        else:
            # default to Canny if unknown
            return cv.Canny(gray_blurred, self.canny_low_threshold, self.canny_high_threshold)

    def _detect_edges_hed(self, img_bgr):
        """
        Perform neural network based HED inference on the input color image.
        """
        if self.hed_net is None:
            if not os.path.isfile(self.HED_PROTOTXT) or not os.path.isfile(self.HED_CAFFEMODEL):
                raise FileNotFoundError("Could not find HED prototxt or caffemodel. Please check paths.")
            # Register custom CropLayer and load the model
            cv.dnn_registerLayer('Crop', CropLayer)
            self.hed_net = cv.dnn.readNetFromCaffe(self.HED_PROTOTXT, self.HED_CAFFEMODEL)

        blob = cv.dnn.blobFromImage(
            img_bgr,
            scalefactor=1.0,
            size=(self.HED_WIDTH, self.HED_HEIGHT),
            mean=(104.00698793, 116.66876762, 122.67891434),
            swapRB=False,
            crop=False)
        self.hed_net.setInput(blob)
        hed = self.hed_net.forward()[0, 0]

        # Resize back to original image shape
        hed_resized = cv.resize(hed, (img_bgr.shape[1], img_bgr.shape[0]))
        # Scale to [0..255]
        if hed_resized.max() > 0:
            hed_resized = hed_resized * (255.0 / hed_resized.max())

        return cv.convertScaleAbs(hed_resized)


def main():
    from glob import glob

    input_folder = 'src/edge_detection/data/images'
    output_folder = 'results/basic/python'
    os.makedirs(output_folder, exist_ok=True)

    extensions = ('*.png', '*.jpg', '*.jpeg', '*.bmp', '*.tiff')
    image_files = [f for ext in extensions for f in glob(os.path.join(input_folder, ext))]
    if not image_files:
        print("No input images found.")
        return

    # Classical methods
    methods = ['canny', 'sobel', 'laplacian', 'prewitt', 'roberts', 'hed']
    detector = EdgeDetector(method="canny")  # reuse instance and update method later

    for img_path in image_files:
        img = cv.imread(img_path)
        if img is None:
            print(f"Warning: Unable to read {img_path}. Skipping.")
            continue

        base_name = os.path.splitext(os.path.basename(img_path))[0]

        for method in methods:
            detector.method = method
            edge_img = detector.detect_edges(img)
            out_path = os.path.join(output_folder, f"{base_name}_{method}.png")
            cv.imwrite(out_path, edge_img)

        print(f"Processed: {base_name}")


if __name__ == '__main__':
    main()