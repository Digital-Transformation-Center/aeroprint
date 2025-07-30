import PrusaLinkPy

# # Replace with your printer's IP address and API key
# PRINTER_IP = "192.168.1.100"
# API_KEY = "your_api_key_here"

def get_printer_status():
    print("Setting up printer..")
    prusaMK4 = PrusaLinkPy.PrusaLinkPy("192.168.8.181", "vWDzCjgQmUxfemt")
    print("Connecting to printer...")
    printer = prusaMK4.get_printer()
    print("Getting printer status...")
    return printer.json()

if __name__ == "__main__":
    status = get_printer_status()
    print("Printer Status:", status)