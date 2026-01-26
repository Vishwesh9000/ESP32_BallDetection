import socket
import io
import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk

class StreamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Live Stream")
        self.root.geometry("640x480")
        
        # Socket Setup
        self.addr = ('192.168.4.1', 8888)
        self.buffer = bytearray()
        self.last_raw_frame = None # Stores raw bytes for saving
        
        # UI Elements
        self.canvas = tk.Label(root, bg = "black", borderwidth=0, highlightthickness=0)
        self.canvas.pack(fill = "both", expand=True)
        
        btn_frame = tk.Frame(root)
        btn_frame.pack(fill="x", side="bottom", pady=10)
        
        tk.Button(btn_frame, text="Save Frame", command=self.save_image).pack(side="left", padx=20, expand=True)
        tk.Button(btn_frame, text="Exit", command=self.quit_app, fg="red").pack(side="right", padx=20, expand=True)


        self.pending_render = False
        self.resize_job_id = None
        self.pil_img = None
        self.root.bind("<Configure>", self.on_resize)
        

        # Connection
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(self.addr)
        self.s.setblocking(False)
        
        self.root.update()
        self.update_stream()

    def on_resize(self, event):
        if self.pending_render == True:
            self.root.after_cancel(self.resize_job_id)
        self.resize_job_id = self.root.after(100, self.renderImg)
        self.pending_render = True


    def renderImg(self):
        if self.pil_img:
            print(f"Rendering Image [len: {len(self.last_raw_frame)}, {self.canvas.winfo_width()=} {self.canvas.winfo_height()=}]")
            w = self.canvas.winfo_width()
            h = self.canvas.winfo_height()
            resized_pil_img = self.pil_img.resize((w, h), Image.Resampling.NEAREST)
            img_tk = ImageTk.PhotoImage(image=resized_pil_img)
            
            self.canvas.config(image=img_tk)
            self.canvas.tk_image = img_tk 
        self.pending_render = False


    def update_stream(self):
        try:
            chunk = self.s.recv(8192)
            if chunk:
                self.buffer.extend(chunk)
            
            # Find JPEG start and end markers
            start = self.buffer.find(b'\xff\xd8')
            end = self.buffer.find(b'\xff\xd9', start)
            
            if start != -1 and end != -1:
                frame_data = self.buffer[start:end+2]
                self.buffer = self.buffer[end+2:]
                self.last_raw_frame = frame_data # Save for file writing
                
                # Decode JPEG bytes directly using Pillow
                self.pil_img = Image.open(io.BytesIO(frame_data))
                if self.pending_render == True:
                    self.root.after_cancel(self.resize_job_id)
                self.renderImg()

        except (BlockingIOError, socket.error):
            pass 

        self.root.after(10, self.update_stream)

    def save_image(self):
        if self.last_raw_frame:
            path = filedialog.asksaveasfilename(defaultextension=".jpg",
                                               filetypes=[("JPEG", "*.jpg")])
            if path:
                with open(path, "wb") as f:
                    f.write(self.last_raw_frame)

    def quit_app(self):
        self.s.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StreamApp(root)
    root.mainloop()