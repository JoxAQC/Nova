import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbUart import YbUart  # 📡 Librería UART de Yahboom

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

# ---- Umbrales LAB para rojo y verde ----
THRESHOLDS = {
    "Rojo":  (0, 66, 7, 127, 3, 127),
    "Verde": (42, 100, -128, -17, 6, 66)
}

def get_closest_rgb(lab_threshold):
    l_center = (lab_threshold[0] + lab_threshold[1]) // 2
    a_center = (lab_threshold[2] + lab_threshold[3]) // 2
    b_center = (lab_threshold[4] + lab_threshold[5]) // 2
    return image.lab_to_rgb((l_center, a_center, b_center))

def init_sensor():
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor

def init_display():
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()

def process_blobs(img, blobs, color):
    for blob in blobs:
        img.draw_rectangle(blob[0:4], color=color, thickness=3)
        img.draw_cross(blob[5], blob[6], color=color, thickness=2)

def draw_fps(img, fps):
    img.draw_string_advanced(0, 0, 30, f'FPS: {fps:.2f}', color=(255, 255, 255))

def main():
    try:
        # ---- Inicializaciones ----
        sensor = init_sensor()
        init_display()
        uart = YbUart(baudrate=115200)  # 🛰️ UART1 predeterminado

        sensor.run()
        clock = time.clock()
        color_map = {name: get_closest_rgb(th) for name, th in THRESHOLDS.items()}

        detected_color = "Nada"

        while True:
            clock.tick()
            img = sensor.snapshot()
            detected_color = "Nada"

            for name, threshold in THRESHOLDS.items():
                blobs = img.find_blobs([threshold], pixels_threshold=200, area_threshold=200)
                if blobs:
                    process_blobs(img, blobs, color_map[name])
                    detected_color = name
                    break

            fps = clock.fps()
            draw_fps(img, fps)

            # Mostrar y enviar información
            print(f"Detectado: {detected_color} | FPS={fps:.2f}")
            uart.send(f"Color detectado: {detected_color}\n")  # Envía al puerto serial (cadena con salto de línea)

            Display.show_image(img)

    except KeyboardInterrupt:
        print("Usuario interrumpió el programa.")
    except Exception as e:
        print(f"Error ocurrido: {e}")
    finally:
        if 'uart' in locals():
            uart.deinit()
        if 'sensor' in locals() and isinstance(sensor, Sensor):
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()

if __name__ == "__main__":
    main()
