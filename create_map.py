import tkinter as tk
from PIL import Image, ImageDraw

class PaintApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Draw Map")
        self.canvas_width = 80
        self.canvas_height = 80

        self.canvas = tk.Canvas(root, bg='black', width=self.canvas_width, height=self.canvas_height)
        self.canvas.pack()

        self.image = Image.new('1', (self.canvas_width, self.canvas_height), 'black')  
        self.draw = ImageDraw.Draw(self.image)

        self.canvas.bind('<ButtonPress-1>', self.start_paint)
        self.canvas.bind('<B1-Motion>', self.paint)

        self.last_x, self.last_y = None, None

        self.button_frame = tk.Frame(root)
        self.button_frame.pack()
        self.save_button = tk.Button(self.button_frame, text='Save as txt', command=self.save_as_txt)
        self.save_button.pack(side='left')
        self.clear_button = tk.Button(self.button_frame, text='Clear', command=self.clear_canvas)
        self.clear_button.pack(side='left')

    def start_paint(self, event):
        self.last_x, self.last_y = event.x, event.y

    def paint(self, event):
        x, y = event.x, event.y
        self.canvas.create_line(self.last_x, self.last_y, x, y, fill='white', width=2)
        self.draw.line((self.last_x, self.last_y, x, y), fill='white', width=2)
        self.last_x, self.last_y = x, y

    def clear_canvas(self):
        self.canvas.delete('all')
        self.image = Image.new('1', (self.canvas_width, self.canvas_height), 'black')
        self.draw = ImageDraw.Draw(self.image)

    def save_as_txt(self):
        pixels = self.image.load()
        width, height = self.image.size

        with open('map.txt', 'w') as f:
            for y in range(height):
                for x in range(width):
                    pixel = pixels[x, y]
                    f.write('1' if pixel else '0')
                f.write('\n')
        print('Saved drawing to map.txt')

if __name__ == '__main__':
    root = tk.Tk()
    app = PaintApp(root)
    root.mainloop()
