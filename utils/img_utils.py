import cv2


def read_image(image_path):
    """
    使用 OpenCV 读取图像并返回图像的 ndarray。

    :param image_path: 图像文件的路径。
    :return: 图像的 ndarray。
    """
    # 使用 OpenCV 读取图像
    image = cv2.imread(image_path)

    # 检查图像是否成功加载
    if image is None:
        raise FileNotFoundError(f"无法加载图像文件: {image_path}")

    return image


def hex_to_rgb(hex_color):
    """
    将十六进制颜色代码转换为RGB三元组。

    参数:
    hex_color (str): 以'#'开头的十六进制颜色代码，例如 '#FF5733'。

    返回:
    tuple: 包含RGB值的三元组，例如 (255, 87, 51)。
    """
    # 确保输入是以'#'开头的7个字符
    if hex_color.startswith('#') and len(hex_color) == 7:
        # 转换为RGB
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        return r, g, b
    else:
        raise ValueError("输入必须是以'#'开头的7个字符，例如 '#RRGGBB'")
