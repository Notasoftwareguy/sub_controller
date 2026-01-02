"""
Received data must be in the format:

NAME:data
"""

class SubData:
    def __init__(self,):
        self.received_data = []
    
    def __iter__(self):
        return self
    
    def __next__(self):
        if not self.received_data:
            raise StopIteration
        return self.received_data.pop(0)

    def add_data(self, received_string):
        delimiter_ind = received_string.index(":")
        self.received_data.append([received_string[:delimiter_ind], received_string[delimiter_ind+1:]])