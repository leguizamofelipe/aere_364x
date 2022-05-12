import threading
import main

def run_repl():
 main.interact(local=globals())
 pass

threading.Thread(target=run_repl).start()