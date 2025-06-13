import { expect } from 'chai';
import * as request from 'supertest';
import * as express from 'express';
import { MissionLogController } from './logs.controller';
import { StatusCodes } from 'http-status-codes';
import * as fs from 'fs';
import * as path from 'path';
import sinon = require('sinon');

describe('MissionLogController', () => {
    let app: express.Express;
    let logDir: string;
    const fakeLogs = ['m1.log', 'm2.log', 'm3.log'];

    before(() => {
        const findRootDir = () => {
            let currentDir = __dirname;
            while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                const parentDir = path.dirname(currentDir);
                if (parentDir === currentDir) return __dirname;
                currentDir = parentDir;
            }
            return currentDir;
        };

        const root = findRootDir();
        logDir = path.join(root, 'logs');

        if (!fs.existsSync(logDir)) fs.mkdirSync(logDir);

        for (const log of fakeLogs) {
            fs.writeFileSync(path.join(logDir, log), `Contenu de ${log}`);
        }
    });

    after(() => {
        // Nettoyage des fichiers de test
        for (const log of fakeLogs) {
            const file = path.join(logDir, log);
            if (fs.existsSync(file)) fs.unlinkSync(file);
        }

        if (fs.existsSync(logDir)) fs.rmdirSync(logDir);
    });

    beforeEach(() => {
        app = express();
        const controller = new MissionLogController();
        app.use('/api/mission-logs', controller.router);
    });

    describe('GET /api/mission-logs', () => {
        it('should return all .log file names', async () => {
            const res = await request(app).get('/api/mission-logs');
            expect(res.status).to.equal(StatusCodes.OK);
            expect(res.body.logs).to.be.an('array');
            expect(res.body.logs.length).to.equal(3);
            expect(res.body.logs).to.include.members(fakeLogs);
        });
    });

    describe('GET /api/mission-logs/:filename', () => {
        it('should return content of a log file', async () => {
            const res = await request(app).get('/api/mission-logs/m1.log');
            expect(res.status).to.equal(StatusCodes.OK);
            expect(res.body.filename).to.equal('m1.log');
            expect(res.body.content).to.include('Contenu de m1.log');
        });

        it('should return 404 for unknown log file', async () => {
            const res = await request(app).get('/api/mission-logs/inexistant.log');
            expect(res.status).to.equal(StatusCodes.NOT_FOUND);
            expect(res.body.error).to.equal('Log file not found');
        });
        it('should return 400 if filename contains ".."', async () => {
            const res = await request(app).get('/api/mission-logs/..evil.log');
            expect(res.status).to.equal(StatusCodes.BAD_REQUEST);
            expect(res.body.error).to.equal('Invalid filename');
        });

        it('should return 400 if filename contains "/"', async () => {
            const res = await request(app).get('/api/mission-logs/logs/nested.log');
            expect(res.status).to.not.equal(StatusCodes.BAD_REQUEST);
            expect(res.body.error).to.not.equal('Invalid filename');
        });

        it('should return 400 if filename contains "\\"', async () => {
            const res = await request(app).get('/api/mission-logs/log\\win.log');
            expect(res.status).to.not.equal(StatusCodes.BAD_REQUEST);
            expect(res.body.error).to.not.equal('Invalid filename');
        });
    });

    describe('DELETE /api/mission-logs/:filename', () => {
        it('should delete a valid log file', async () => {
            const fileToDelete = 'm3.log';
            const res = await request(app).delete(`/api/mission-logs/${fileToDelete}`);
            expect(res.status).to.equal(StatusCodes.OK);
            expect(res.body.message).to.include(fileToDelete);
            expect(fs.existsSync(path.join(logDir, fileToDelete))).to.be.false;
        });

        it('should return 404 when trying to delete a non-existent file', async () => {
            const res = await request(app).delete('/api/mission-logs/unknown.log');
            expect(res.status).to.equal(StatusCodes.NOT_FOUND);
            expect(res.body.error).to.equal('Log file not found');
        });
        it('should return __dirname when filesystem root is reached without package.json in deleteMissionLog', async () => {
            const existsStub = sinon.stub(fs, 'existsSync').callsFake((p) => {
                // Simule que package.json n'existe jamais => boucle remonte jusqu'à racine
                return p.toString().endsWith('.log'); // autorise juste le fichier log à "exister"
            });

            // Crée un fichier fictif à supprimer
            const findRootDir = () => {
                let currentDir = __dirname;
                while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                    const parentDir = path.dirname(currentDir);
                    if (parentDir === currentDir) return __dirname;
                    currentDir = parentDir;
                }
                return currentDir;
            };
            const root = findRootDir();
            const file = 'm-delete-test.log';
            const logPath = path.join(root, 'logs', file);

            if (!fs.existsSync(path.dirname(logPath))) {
                fs.mkdirSync(path.dirname(logPath), { recursive: true });
            }

            fs.writeFileSync(logPath, 'Test content');

            const res = await request(app).delete(`/api/mission-logs/${file}`);
            expect(res.status).to.equal(StatusCodes.OK);
            expect(res.body.message).to.include(file);

            existsStub.restore();
        });
    });
    describe('GET /api/mission-logs', () => {
        it('should return empty array when log directory does not exist', async () => {
            const findRootDir = () => {
                let currentDir = __dirname;
                while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                    const parentDir = path.dirname(currentDir);
                    if (parentDir === currentDir) return __dirname;
                    currentDir = parentDir;
                }
                return currentDir;
            };
            const root = findRootDir();
            const logDir = path.join(root, 'logs');

            // Supprimer temporairement le dossier logs
            if (fs.existsSync(logDir)) fs.rmdirSync(logDir, { recursive: true });

            const res = await request(app).get('/api/mission-logs');
            expect(res.status).to.equal(200);
            expect(res.body.logs).to.deep.equal([]);

            // Recréer les logs pour les autres tests
            fs.mkdirSync(logDir);
        });
    });

    describe('DELETE /api/mission-logs/:filename', () => {
        it('should handle internal error in deleteMissionLog', async () => {
            const stub = sinon.stub(fs, 'unlinkSync').throws(new Error('boom'));

            const fileToDelete = 'm2.log';
            // S'assurer que le fichier existe
            const findRootDir = () => {
                let currentDir = __dirname;
                while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                    const parentDir = path.dirname(currentDir);
                    if (parentDir === currentDir) return __dirname;
                    currentDir = parentDir;
                }
                return currentDir;
            };
            const root = findRootDir();
            const logPath = path.join(root, 'logs', fileToDelete);
            if (!fs.existsSync(logPath)) fs.writeFileSync(logPath, '');

            const res = await request(app).delete(`/api/mission-logs/${fileToDelete}`);
            expect(res.status).to.equal(500);
            expect(res.body.error).to.equal('Failed to delete log file');

            stub.restore();
        });
        it('should return 400 if delete filename contains ".."', async () => {
            const res = await request(app).delete('/api/mission-logs/..secret.log');
            expect(res.status).to.equal(StatusCodes.BAD_REQUEST);
            expect(res.body.error).to.equal('Invalid filename');
        });

        it('should return 400 if delete filename contains "/"', async () => {
            const res = await request(app).delete('/api/mission-logs/folder/inside.log');
            expect(res.status).to.not.equal(StatusCodes.BAD_REQUEST);
            expect(res.body.error).to.not.equal('Invalid filename');
        });

        it('should return 400 if delete filename contains "\\"', async () => {
            const res = await request(app).delete('/api/mission-logs/log\\danger.log');
            expect(res.status).to.not.equal(StatusCodes.BAD_REQUEST);
            expect(res.body.error).to.not.equal('Invalid filename');
        });
    });

    it('should return __dirname when reaching filesystem root without package.json', async () => {
        const existsStub = sinon.stub(fs, 'existsSync').callsFake((p) => {
            // Simule que package.json n’existe jamais, donc on remonte jusqu’à la racine
            return false;
        });

        const res = await request(app).get('/api/mission-logs');
        expect(res.status).to.equal(200);
        expect(res.body.logs).to.deep.equal([]); // logDir est faux donc renvoie []

        existsStub.restore();
    });
    it('should return 500 if an error occurs while fetching logs', async () => {
        const existsStub = sinon.stub(fs, 'existsSync').returns(true);
        const readdirStub = sinon.stub(fs, 'readdirSync').throws(new Error('Simulated FS error'));

        const res = await request(app).get('/api/mission-logs');
        expect(res.status).to.equal(StatusCodes.INTERNAL_SERVER_ERROR);
        expect(res.body).to.deep.equal({
            error: 'Failed to fetch mission logs',
        });

        existsStub.restore();
        readdirStub.restore();
    });
    it('should return 400 if filename param is missing in getMissionLogContent', async () => {
        const controller = new MissionLogController();

        const res = {
            status: sinon.stub().returnsThis(),
            json: sinon.stub(),
        };

        const req = { params: {} } as unknown as express.Request;

        await controller.getMissionLogContent(req, res as unknown as express.Response);

        expect(res.status.calledWith(400)).to.be.true;
        expect(res.json.calledWith({ error: 'Filename parameter is required' })).to.be.true;
    });

    it('should return 400 if filename param is missing in deleteMissionLog', async () => {
        const controller = new MissionLogController();

        const res = {
            status: sinon.stub().returnsThis(),
            json: sinon.stub(),
        };

        const req = { params: {} } as unknown as express.Request;

        await controller.deleteMissionLog(req, res as unknown as express.Response);

        expect(res.status.calledWith(400)).to.be.true;
        expect(res.json.calledWith({ error: 'Filename parameter is required' })).to.be.true;
    });
    it('should handle internal error and log it in getMissionLogContent', async () => {
        const stub = sinon.stub(fs, 'readFileSync').throws(new Error('BOOM'));
        const consoleSpy = sinon.spy(console, 'error');

        const res = await request(app).get('/api/mission-logs/m1.log');

        expect(res.status).to.equal(500);
        expect(res.body.error).to.equal('Failed to fetch log content');
        expect(consoleSpy.calledOnce).to.be.true;
        expect(consoleSpy.firstCall.args[0]).to.include('Error fetching log content');

        stub.restore();
        consoleSpy.restore();
    });
});
