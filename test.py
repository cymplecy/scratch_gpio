val = self.pinRead(pin)
        time.sleep(0.02)
        if val == self.pinRead(pin):
            self.pinCount[pin] += (self.countDirection[pin] * 1) # inc or dec count based on direction required
            self.pinLastState[pin] = int(val)