import cv2
import os
import glob
import tqdm
from utils.os_utils import extract_number_single


def images_to_video(image_folder, video_name, fps):
    images = glob.glob(os.path.join(image_folder, '*.png'))
    images = sorted(images, key=extract_number_single)
    frame = cv2.imread(images[0])
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, cv2.VideoWriter.fourcc(*'mp4v'), fps, (width, height))

    image_iter = 0
    for image in tqdm.tqdm(images):
        video.write(cv2.imread(image))
        image_iter += 1

    cv2.destroyAllWindows()
    video.release()


if __name__ == '__main__':
    image_dir = 'C:\\Users\\34995\\Desktop\\Phanes\\fast_demo\\fast_demo_images'
    images_to_video(image_dir, 'output_video.mp4', 1)
