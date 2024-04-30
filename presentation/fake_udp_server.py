import socket

def tcp_client(host='10.205.3.76', port=3434):
    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Connect to the server
        s.connect((host, port))
        for i in range(100):
            # Get input from the user
            message = input("Let's make a natural pause: ")
            # Send the message to the server
            s.sendall(message.encode())
            
            # Print message sent
            print(f"Message sent: {message}")
 

if __name__ == "__main__":
    tcp_client()
