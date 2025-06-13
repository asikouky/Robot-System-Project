import { expect } from 'chai';
import * as sinon from 'sinon';
import { Timer } from '@app/classes/timer/timer';

describe('Timer', () => {
    let timer: Timer & NodeJS.EventEmitter;
    let clock: sinon.SinonFakeTimers;

    beforeEach(() => {
        clock = sinon.useFakeTimers();
        timer = new Timer();
    });

    afterEach(() => {
        clock.restore();
    });

    it('devrait émettre l’événement "timeUpdated" avec la bonne valeur', () => {
        const spy = sinon.spy();
        timer.on('timeUpdated', spy); 

        timer.startTimer(10);
        clock.tick(1000);

        expect(spy.calledOnce).to.be.true;
        expect(spy.calledWith(11)).to.be.true;
    });
    

    it('devrait remettre le compteur à 0 avec stopTimer', () => {
        timer.startTimer(10);
        timer.stopTimer();

        
        const spy = sinon.spy();
        timer.on('timeUpdated', spy);
        timer.stopTimer(); 

        expect(spy.called).to.be.false;
    });
});
