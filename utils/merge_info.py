import os
import cv2
import json
import numpy as np
from tqdm import tqdm
from PIL import Image, ImageDraw, ImageFont

from utils.img_utils import read_image, hex_to_rgb
from llm_module.detection import split_string


font_path = "times.ttf"
info_dir = 'C:\\Users\\34995\\Desktop\\Phanes\\fast_demo'
save_dir = 'C:\\Users\\34995\\Desktop\\Phanes\\fast_demo\\fast_demo_images'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
for i in tqdm(range(11)):
    image_path = os.path.join(info_dir, 'figs', '{}.png'.format(i))
    input_path = os.path.join(info_dir, 'robot_log', 'step_{}.txt'.format(i))
    detection_path = os.path.join(info_dir, 'detection', 'detection_{}.json'.format(i))
    solution_path = os.path.join(info_dir, 'solution', 'solution_{}.json'.format(i))
    save_path = os.path.join(save_dir, '{}.png'.format(i))

    with open(input_path, 'r') as file:
        input_str = file.read()
    with open(detection_path, 'r') as file:
        detection_dict = json.load(file)
    try:
        with open(solution_path, 'r') as file:
            solution_dict = json.load(file)
    except FileNotFoundError:
        solution_dict = {'short_term_solution': '', 'long_term_solution': ''}

    # -------------------------------------------------------------------------------------

    image_array = read_image(image_path)
    image = 255 * np.ones((940, 1220, 3)).astype(image_array.dtype)
    image[:600, :600, :] = image_array
    image[:100, 600:, :] = hex_to_rgb('#BFBFBF')[::-1]
    image[120: 380, 600:, :] = hex_to_rgb('#F4B183')[::-1]
    image[400: 600, 600:, :] = hex_to_rgb('#FFD966')[::-1]
    image[270: 290, 720:, :] = 255
    image[:, 700: 720, :] = 255
    image[620:, :, :] = hex_to_rgb('#9DC3E6')[::-1]
    image[600:, 100: 120, :] = 255
    image[600:, 220: 240, :] = 255
    image[770: 790, 120:, :] = 255

    pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

    draw = ImageDraw.Draw(pil_image)

    # -------------------------------------------------------------------------------------

    font_size = 20
    font = ImageFont.truetype(font_path, font_size)

    text = 'INPUT'
    text_bbox = draw.textbbox((0, 0), text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (50, 650)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, text, font=font, fill="black")

    text = 'FIND'
    text_bbox = draw.textbbox((0, 0), text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (250, 650)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, text, font=font, fill="black")

    text = 'ANALYZE'
    text_bbox = draw.textbbox((0, 0), text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (500, 650)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, text, font=font, fill="black")

    text = 'SOLVE'
    text_bbox = draw.textbbox((0, 0), text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (780, 50)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, text, font=font, fill="black")

    text = 'SHORT'
    text_bbox = draw.textbbox((0, 0), text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (695, 170)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, text, font=font, fill="black")

    text = 'LONG'
    text_bbox = draw.textbbox((0, 0), text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (865, 170)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, text, font=font, fill="black")

    # -------------------------------------------------------------------------------------
    # input

    font_size = 15
    font = ImageFont.truetype(font_path, font_size)

    input_text = split_string(input_str, 80)
    text_bbox = draw.textbbox((0, 0), input_text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (50, 970)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, input_text, font=font, fill="black")

    # -------------------------------------------------------------------------------------
    # report

    report_text = detection_dict['report']
    report_text = split_string(report_text, 70)
    text_bbox = draw.textbbox((0, 0), report_text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (195, 970)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, report_text, font=font, fill="black")

    # -------------------------------------------------------------------------------------
    # reason

    report_text = detection_dict['reason']
    report_text = split_string(report_text, 70)
    text_bbox = draw.textbbox((0, 0), report_text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (500, 970)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, report_text, font=font, fill="black")

    # -------------------------------------------------------------------------------------
    # solve-short

    short_solution_text = solution_dict['short_term_solution']
    short_solution_text = split_string(short_solution_text, 125)
    text_bbox = draw.textbbox((0, 0), short_solution_text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (695, 730)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, short_solution_text, font=font, fill="black")

    # -------------------------------------------------------------------------------------
    # solve-long

    long_solution_text = solution_dict['long_term_solution']
    long_solution_text = split_string(long_solution_text, 125)
    text_bbox = draw.textbbox((0, 0), long_solution_text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (865, 730)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, long_solution_text, font=font, fill="black")

    # -------------------------------------------------------------------------------------
    # score

    font_size = 30
    font = ImageFont.truetype(font_path, font_size)

    score_text = str(detection_dict['score'])
    score_text = split_string(score_text, 70)
    text_bbox = draw.textbbox((0, 0), score_text, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    center = (335, 970)

    position = (center[1] - text_width / 2, center[0] - text_height / 2)
    draw.text(position, score_text, font=font, fill="black")

    # -------------------------------------------------------------------------------------

    image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
    border_size = 20
    white = [255, 255, 255]
    image = cv2.copyMakeBorder(image, border_size, border_size,
                               border_size, border_size, cv2.BORDER_CONSTANT, value=white)
    cv2.imwrite(save_path, image)
    # cv2.imshow('Image', image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
