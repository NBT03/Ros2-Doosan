import socket

HOST = '0.0.0.0'  # Lắng nghe trên tất cả các IP của máy
PORT = 12345      # Cổng kết nối

# Tạo socket TCP
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))  # Gắn kết socket với địa chỉ và cổng
server_socket.listen(10)
print(f"🔵 Server đang chạy trên {HOST}:{PORT}...")

while True:
    conn, addr = server_socket.accept()  # Chấp nhận kết nối
    print(f"🟢 Nhận kết nối từ {addr}")

    data = conn.recv(1024).decode()  # Nhận dữ liệu từ client
    if not data:
        break
    print(f"📩 Dữ liệu nhận được: {data}")

    conn.sendall("finish".encode())  # Gửi phản hồi
    # conn.close()  # Đóng kết nối
