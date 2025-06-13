import { EventEmitter } from "events";

export class Timer extends EventEmitter {
    private counter: number = 0;
    private isRunning: boolean = false;

    constructor() {
        super();
    }

    private tick() {
        if (!this.isRunning) return;
        this.counter++;
        this.emit("timeUpdated", this.counter);
        setTimeout(() => this.tick(), 1000); 
    }

    startTimer(startValue: number) {
        if (this.isRunning) return;
        this.counter = startValue;
        this.isRunning = true;
        this.tick(); 
    }

    stopTimer() {
        this.isRunning = false;
    }

    getTime(): number {
        return this.counter;
    }
}
