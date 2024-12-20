from config import debug

def log(message):
    if debug:
        print(message)