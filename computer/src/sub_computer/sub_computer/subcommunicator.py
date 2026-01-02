import socket
import threading
from typing import Callable, Optional


class SubCommunicator:
    """
    A class for sending and receiving strings over TCP ethernet.
    Supports both server (listen) and client (connect) modes.
    """
    
    def __init__(self, host: str = 'localhost', port: int = 5000, encoding: str = 'utf-8'):
        """
        Initialize the communicator.
        
        Args:
            host: IP address or hostname to bind/connect to
            port: Port number for communication
            encoding: Character encoding for strings (default: utf-8)
        """
        self.host = host
        self.port = port
        self.encoding = encoding
        self.socket = None
        self.connection = None
        self.is_listening = False
        self.receive_callback: Optional[Callable[[str], None]] = None
        self.receive_thread = None
        
    def start_server(self, backlog: int = 1) -> None:
        """
        Start listening for incoming connections.
        
        Args:
            backlog: Number of queued connections before refusing new ones
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(backlog)
        self.is_listening = True
        print(f"Server listening on {self.host}:{self.port}")
        
        # Accept connection in a thread
        self.receive_thread = threading.Thread(target=self._accept_connection, daemon=True)
        self.receive_thread.start()
    
    def _accept_connection(self) -> None:
        """Accept incoming connections in a loop and spawn handlers."""
        try:
            while self.is_listening:
                try:
                    conn, addr = self.socket.accept()
                except OSError:
                    # Socket closed or interrupted
                    break

                print(f"Connected from {addr}")

                # If an existing connection exists, close it (single-connection model)
                if self.connection is not None:
                    try:
                        self.connection.close()
                    except Exception:
                        pass

                # Handle this client in its own thread so accept can continue
                handler = threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True)
                handler.start()
        except Exception as e:
            print(f"Error accepting connections: {e}")

    def _handle_client(self, conn: socket.socket, addr) -> None:
        """Per-connection handler which receives data until the client disconnects."""
        # Make this connection the active connection for send()
        self.connection = conn
        try:
            self._receive_loop(conn)
        finally:
            try:
                conn.close()
            except Exception:
                pass
            if self.connection is conn:
                self.connection = None
            print(f"Disconnected from {addr}")
    
    def connect(self) -> bool:
        """
        Connect to a server.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connection = self.socket
            print(f"Connected to {self.host}:{self.port}")
            
            # Start receiving in a thread
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            return True
        except Exception as e:
            # print(f"Error connecting: {e}")
            return False
    
    def _receive_loop(self, conn: Optional[socket.socket] = None) -> None:
        """Continuously receive messages from the provided connection.

        If `conn` is None, uses `self.connection` (client-mode).
        """
        if conn is None:
            conn = self.connection
        if conn is None:
            return

        while True:
            try:
                data = conn.recv(4096)
                if not data:
                    print("Connection closed")
                    break

                message = data.decode(self.encoding)
                print(message)
                if self.receive_callback:
                    self.receive_callback(message)
            except Exception as e:
                print(f"Error receiving: {e}")
                break
            # If server is shutting down, exit
            if not self.is_listening:
                break
            
    def send(self, message: str) -> bool:
        """
        Send a string message.
        
        Args:
            message: String to send
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.connection:
            print("No active connection")
            return False
        
        try:
            self.connection.sendall(message.encode(self.encoding))
            return True
        except Exception as e:
            print(f"Error sending: {e}")
            return False
    
    def set_receive_callback(self, callback: Callable[[str], None]) -> None:
        """
        Set a callback function to be called when a message is received.
        
        Args:
            callback: Function that takes a string as argument
        """
        self.receive_callback = callback
    
    def close(self) -> None:
        """Close all connections."""
        if self.connection:
            self.connection.close()
        if self.socket:
            self.socket.close()
        self.is_listening = False
        print("Connection closed")




