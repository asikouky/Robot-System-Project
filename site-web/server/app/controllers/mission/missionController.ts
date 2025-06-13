
import { Router, Request, Response } from 'express';
import { Service } from 'typedi';
import { connectToDatabase } from '@app/services/mangodb/missionMangodb';
import { ObjectId } from 'mongodb';

@Service()
export class MissionController {
  public router: Router;

  constructor() {
    this.router = Router();
    this.configureRoutes();
  }


  private configureRoutes(): void {
    this.router.get('/', this.getAllMissions);
    this.router.delete('/:id', this.deleteMission);
   
  }

  /**
   * GET /api/mission
   */
  public getAllMissions = async (req: Request, res: Response): Promise<void> => {
    try {
      const db = await connectToDatabase();
      const missions = await db.collection('missions')
        .find()
        .sort({ createdAt: -1 })
        .toArray();

      res.status(200).json(missions);
    } catch (error) {
      console.error('[MissionController] Error fetching missions:', error);
      res.status(500).json({ error: 'Erreur de récupération des missions' });
    }
  };

  /**
   * DELETE /api/mission/:id
   */
  public deleteMission = async (req: Request, res: Response): Promise<void> => {
    try {
      const { id } = req.params;

      if (!ObjectId.isValid(id)) {
        res.status(400).json({ error: 'ID invalide' });
        return;
      }

      const db = await connectToDatabase();
      const result = await db.collection('missions').deleteOne({ _id: new ObjectId(id) });

      if (result.deletedCount === 1) {
        res.status(200).json({ message: 'Mission supprimée' });
      } else {
        res.status(404).json({ error: 'Mission non trouvée' });
      }
    } catch (error) {
      console.error('[MissionController] Erreur suppression mission:', error);
      res.status(500).json({ error: 'Erreur serveur' });
    }
  };
}

