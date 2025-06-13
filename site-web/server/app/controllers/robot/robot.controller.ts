import { Router, Request, Response } from 'express';
import { StatusCodes } from 'http-status-codes';
import { Service } from 'typedi';
import { exec } from 'child_process';
import * as fs from 'fs';
import * as path from 'path';

@Service()
export class RobotController {
    public router: Router;

    constructor() {
        this.router = Router();
        this.configureRouter();
    }

    private configureRouter(): void {
      
        this.router.post('/start-robot', this.startRobot.bind(this));

        
        this.router.get('/getCode', this.getCode.bind(this));

        
        this.router.post('/saveCode', this.saveCode.bind(this));

        this.router.post('/restartGazebo', this.restartGazebo.bind(this));
    }

    private startRobot(req: Request, res: Response): void {
        const robotIp = req.body.ip; 
        if (!robotIp) {
            res.status(StatusCodes.BAD_REQUEST).json({ error: 'Aucune adresse IP fournie.' });
        }

        console.log(`[START_ROS] Lancement de ROS sur ${robotIp}`);


        const sshCommand = `ssh -o StrictHostKeyChecking=no equipe104@${robotIp} 'bash /home/equipe104/INF3995-104/robot_ws/build_and_launch.sh'`;

        exec(sshCommand, (error, stdout, stderr) => {
            if (error) {
                console.error(`[ROS] Erreur: ${error.message}`);
                return res.status(StatusCodes.INTERNAL_SERVER_ERROR).json({ error: error.message });
            }

            console.log(`[ROS] Sortie du script :`, stdout);
            return res.json({ output: stdout });
        });
    }

    private getCode(req: Request, res: Response): void {
        const codePath = path.join(
            __dirname,
            '..', '..', '..', '..', '..', '..','..',
            'robot_ws',
            'src',
            'limo_controller',
            'limo_controller',
            'move_limo.py'
        );
        console.log('[DEBUG] __dirname:', __dirname);
        console.log('[DEBUG] Résolu =>', path.resolve(codePath));
        console.log('[DEBUG] Fichier existe ?', fs.existsSync(codePath));

        try {
            const code = fs.readFileSync(codePath, 'utf8');
            res.send(code);
        } catch (error: any) {
            console.error('[ERREUR getCode]', error.message);
            res.status(StatusCodes.INTERNAL_SERVER_ERROR).json({ error: 'Erreur lors de la lecture du fichier.' });
        }
    }

    private saveCode(req: Request, res: Response): void {
        const { code } = req.body;
        const codePath = path.join(
            __dirname,
            '..', '..', '..', '..', '..', '..','..',
            'robot_ws',
            'src',
            'limo_controller',
            'limo_controller',
            'move_limo.py'
        );
        try {
            fs.writeFileSync(codePath, code); 
            res.send({ message: 'Code sauvegardé avec succès' });
        } catch (error) {
            res.status(StatusCodes.INTERNAL_SERVER_ERROR).json({ error: 'Erreur lors de la sauvegarde du fichier.' });
        }
    }

    private restartGazebo(req: Request, res: Response): void {
        const scriptPath = path.resolve(__dirname, '../robot/restart_gazebo.sh');
    
        exec(scriptPath, (err, stdout, stderr) => {
            if (err) {
                console.error('[ERREUR] Gazebo restart :', stderr);
                return res.status(StatusCodes.INTERNAL_SERVER_ERROR).json({ error: stderr });
            }
    
            console.log('[INFO] Gazebo relancé :', stdout);
            return res.send({ message: 'Gazebo redémarré avec succès', output: stdout });
        });
    }
    
}
