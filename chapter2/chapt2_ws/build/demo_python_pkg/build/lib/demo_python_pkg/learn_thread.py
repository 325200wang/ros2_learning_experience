import threading
import requests

class Download:
    def download(self, url, callback):
        print("Starting download from:", url)
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback(url, response.text)
    
    def start_download(self, url, callback):
        thread = threading.Thread(target=self.download, args=(url, callback))
        thread.start()

def download_finish_callback(url, content):
    print(f"Download finished from: {url}, Content: {content[:10]}...")

def main():
    d = Download()
    d.start_download("http://localhost:8000/novel1.txt", download_finish_callback)
    d.start_download("http://localhost:8000/novel2.txt", download_finish_callback)
    d.start_download("http://localhost:8000/novel3.txt", download_finish_callback)