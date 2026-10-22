import socket
import io
import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk

# Message Protocol:
# b"MS" (Message Start) + 0x01 (Log Message) | 0x02 (Image Buffer) + 4 bytes (Length) + Payload (Log Message (null-terminated) | JPEG Image Bytes (0xFF 0xD8 ... 0xFF 0xD9))
log_message_prefix = b"0x01"
img_buffer_prefix = b"0x02"


class StreamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Live Stream")
        
        # Socket Setup
        self.addr = ('192.168.4.1', 8888)
        self.buffer = bytearray()
        self.last_raw_frame = None # Stores raw bytes for downloading
        
        # UI Elements
        self.vid_frame = tk.Frame(root, bg='black', width=320, height=240)
        self.vid_frame.pack(fill='both', expand=True)
        self.vid_frame.pack_propagate(False)

        self.canvas = tk.Label(self.vid_frame, bg = "black")
        self.canvas.place(relx=0.5, rely=0.5, anchor="center")
        
        self.btn_frame = tk.Frame(root, height=50)
        self.btn_frame.pack(fill="x", side="bottom")
        self.btn_frame.pack_propagate(False)


        self.logs = []
        self.max_logs = 5
        self.log_label = tk.Label(self.root, text="Log: ", height=self.max_logs)
        self.log_label.pack(side="bottom", fill="x")
        tk.Button(self.btn_frame, text="Save Frame", command=self.save_image).pack(side="left", expand=True, fill="both")
        tk.Button(self.btn_frame, text="Revert 320x240", command=self.reset_size).pack(side="left", expand=True, fill="both")
        tk.Button(self.btn_frame, text="Exit", command=self.quit_app, fg="red").pack(side="left", expand=True,  fill="both")


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

    def add_log(self, message):
        self.logs.append(message)
        if len(self.logs) > self.max_logs:
            self.logs.pop(0)
        self.log_label.config(text="\n".join(self.logs))

    def reset_size(self):
        self.root.geometry("320x290")
        self.renderImg


    def on_resize(self, event):
        if self.pending_render == True:
            self.root.after_cancel(self.resize_job_id)
        self.resize_job_id = self.root.after(100, self.renderImg)
        self.pending_render = True


    def renderImg(self):
        if self.pil_img:
            print(f"Rendering Image [len: {len(self.last_raw_frame)}, {self.canvas.winfo_width()=} {self.canvas.winfo_height()=}]")
            self.vid_frame.update_idletasks()
            w = self.vid_frame.winfo_width()
            h = self.vid_frame.winfo_height()
            if w/h > 320/240:
                w = (h*320)//240
            else:
                h = (w*240)//320
            resized_pil_img = self.pil_img.resize((w, h), Image.Resampling.NEAREST)
            img_tk = ImageTk.PhotoImage(image=resized_pil_img)
            
            self.canvas.config(image=img_tk)
            self.canvas.tk_image = img_tk 
        self.pending_render = False


    def update_stream(self):
        self.root.after(10, self.update_stream)
        new_data = False
        while True:
            try:
                chunk = self.s.recv(8192)
                if not chunk:
                    break
            
                self.buffer.extend(chunk)
                new_data = True
            except (BlockingIOError, socket.error):
                break 

        if not new_data:
            return
        
        while True:
            if len(self.buffer) < 7:
                break
            message_start = self.buffer.find(b'MS')
            if message_start == -1:
                print("No message start found, clearing buffer")
                print(f"Discarded: {self.buffer[:-1]}")
                self.buffer = self.buffer[-1:]  # Keep the last byte in case it's part of "MS"
                break

            if message_start > 0:
                print("Clearing buffer before start")
                print(f"Discarded: {self.buffer[:message_start]}")
            self.buffer = self.buffer[message_start:]

            if len(self.buffer) < 7:  # Minimum length for a valid message
                break

            if self.buffer[2] not in (0x01, 0x02):
                print(f"Invalid message type: {self.buffer[2]}, looking for next 'MS'")
                print(f"Discarded: {self.buffer[:2]}")
                self.buffer = self.buffer[2:]  # Remove the "MS" and look for the next "MS"
                continue

            length = int.from_bytes(self.buffer[3:7], byteorder='little')
            if length > 100_000 or length < 1:
                print(f"Invalid message length: {length}, looking for next 'MS'")
                print(f"Discarded: {self.buffer[:2]}")
                self.buffer = self.buffer[2:]  # Remove the "MS" and look for the next "MS"
                continue
            
            if self.buffer[2] == 0x01:  # Log Message
                # print("Received log message")
                if len(self.buffer) < 7 + length:
                    break
                
                if self.buffer[7+length-1] != 0:  # Check for null terminator
                    print(f"Log message not null-terminated, discarding")
                    print(f"Discarded: {self.buffer[:2]}")
                    self.buffer = self.buffer[2:]  # Remove the "MS" and look for the next "MS"
                    continue
                
                message = self.buffer[7:7+length-1].decode('utf-8')
                print(f"[LOG]: {message}")
                self.add_log(message)
                self.buffer = self.buffer[7+length:]  # Remove the processed message from the buffer

            else:  # Image Buffer
                if length < 4:
                    print(f"Invalid image length: {length}, discarding buffer")
                    print(f"Discarded: {self.buffer[:2]}")
                    self.buffer = self.buffer[2:]  # Remove the "MS" and look for the next "MS"
                    continue
                
                if len(self.buffer) < 7 + length:
                    break

                frame_data = self.buffer[7:7+length]
                if frame_data[:2] != b'\xff\xd8' or frame_data[-2:] != b'\xff\xd9':
                    print(f"Invalid JPEG frame received, discarding")
                    print(f"Discarded: {self.buffer[:2]}")
                    self.buffer = self.buffer[2:]  # Remove the "MS" and look for the next "MS"
                    continue
                self.buffer = self.buffer[7+length:]  # Remove the processed frame from the buffer
                self.last_raw_frame = frame_data # Save for file writing
                
                # Decode JPEG bytes directly using Pillow
                self.pil_img = Image.open(io.BytesIO(frame_data))
                if self.pending_render == True:
                    self.root.after_cancel(self.resize_job_id)
                self.renderImg()


            # # Find JPEG start and end markers
            # start = self.buffer.find(b'\xff\xd8')
            # end = self.buffer.find(b'\xff\xd9', start)
            
            # if start != -1 and end != -1:
            #     frame_data = self.buffer[start:end+2]
            #     self.buffer = self.buffer[end+2:]
            #     self.last_raw_frame = frame_data # Save for file writing
                
            #     # Decode JPEG bytes directly using Pillow
            #     self.pil_img = Image.open(io.BytesIO(frame_data))
            #     if self.pending_render == True:
            #         self.root.after_cancel(self.resize_job_id)
            #     self.renderImg()



    def save_image(self):
        if self.last_raw_frame:
            save_frame = self.last_raw_frame
            path = filedialog.asksaveasfilename(defaultextension=".jpg",
                                               filetypes=[("JPEG", "*.jpg")])
            if path:
                with open(path, "wb") as f:
                    f.write(save_frame)

    def quit_app(self):
        self.s.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StreamApp(root)
    root.mainloop()