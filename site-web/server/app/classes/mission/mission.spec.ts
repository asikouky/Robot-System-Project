import { expect } from 'chai';
import * as sinon from 'sinon';
import { Mission } from '@app/classes/mission/mission';
import { Timer } from '@app/classes/timer/timer';
import { MissionStatus } from '@common/action.robot';
import { ModeMission } from '@common/events';

describe('Mission', () => {
    let mission: Mission;
    let timerStub: sinon.SinonStubbedInstance<Timer>;

    beforeEach(() => {
        timerStub = sinon.createStubInstance(Timer);
        mission = new Mission('192.168.0.123', ModeMission.PHYSICAL_ROBOT);
        mission.timer = timerStub as unknown as Timer; // injection du stub
    });

    it('devrait initialiser avec les bonnes valeurs par défaut', () => {
        expect(mission.ipPrimary).to.equal('192.168.0.123');
        expect(mission.status).to.equal(MissionStatus.IDLE);
        expect(mission.timeLeft).to.equal(0);
        expect(mission.active).to.be.false;
    });

    it('devrait passer en IN_MISSION et appeler startTimer lors du démarrage', () => {
        mission.timeLeft = 30;
        mission.start();

        expect(mission.status).to.equal(MissionStatus.IN_MISSION);
        expect(mission.active).to.be.true;
        expect(timerStub.startTimer.calledOnceWithExactly(30)).to.be.true;
    });

    it('devrait repasser en IDLE et appeler stopTimer lors de l\'arrêt', () => {
        mission.start(); // simulate start before stop
        mission.stop();

        expect(mission.status).to.equal(MissionStatus.IDLE);
        expect(mission.active).to.be.false;
        expect(timerStub.stopTimer.calledOnce).to.be.true;
    });

});
