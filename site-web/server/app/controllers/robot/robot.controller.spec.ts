import { expect } from 'chai';
import * as sinon from 'sinon';
import * as request from 'supertest';
import * as express from 'express';
import { RobotController } from './robot.controller';
import { StatusCodes } from 'http-status-codes';
import * as child_process from 'child_process';

describe('RobotController', () => {
  let app: express.Express;
  let execStub: sinon.SinonStub;

  beforeEach(() => {
    app = express();
    app.use(express.json());
    const controller = new RobotController();
    app.use('/api', controller.router);

    execStub = sinon.stub(child_process, 'exec');
  });

  afterEach(() => {
    sinon.restore();
  });

  it('should return 400 if no IP is provided', async () => {
    const res = await request(app).post('/api/start-robot').send({});
    expect(res.status).to.equal(StatusCodes.BAD_REQUEST);
    expect(res.body.error).to.equal('Aucune adresse IP fournie.');
  });

  it('should execute SSH command and return output on success', async () => {
    execStub.yields(null, 'ROS launched!', '');

    const res = await request(app).post('/api/start-robot').send({ ip: '192.168.0.10' });
    expect(res.status).to.equal(StatusCodes.OK);
    expect(execStub.calledOnce).to.be.true;
    expect(res.body.output).to.equal('ROS launched!');
  });

  it('should return 500 if SSH command fails', async () => {
    execStub.yields(new Error('SSH failed'), '', '');

    const res = await request(app).post('/api/start-robot').send({ ip: '192.168.0.10' });
    expect(res.status).to.equal(StatusCodes.INTERNAL_SERVER_ERROR);
    expect(res.body.error).to.equal('SSH failed');
  });
});

