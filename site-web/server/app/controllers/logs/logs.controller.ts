import { Request, Response, Router } from 'express';
import * as fs from 'fs';
import * as path from 'path';
import { Service } from 'typedi';

@Service()
export class MissionLogController {
    public router: Router;

    constructor() {
        this.router = Router();
        this.configureRoutes();
    }

    private configureRoutes(): void {
        this.router.get('/', this.getMissionLogs);
        this.router.get('/:filename', this.getMissionLogContent);
        this.router.delete('/:filename', this.deleteMissionLog);
        this.router.get('/by-mission/:missionId', this.getLogByMissionId);
    }

    /**
     * Get all mission log file names
     * 
     * @param req Express Request object
     * @param res Express Response object
     * @returns Array of mission log file names
     */
    public getMissionLogs = async (req: Request, res: Response): Promise<void> => {
        try {
            // Find the server root directory (same approach as in MissionRoomService)
            const findRootDir = () => {
                let currentDir = __dirname;
                while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                    const parentDir = path.dirname(currentDir);
                    if (parentDir === currentDir) {
                        // We've reached the filesystem root without finding package.json
                        return __dirname; // Fall back to current directory
                    }
                    currentDir = parentDir;
                }
                return currentDir; // This is the directory containing package.json
            };
        
            const serverRoot = findRootDir();
            const logDir = path.join(serverRoot, 'logs');
            
            // Check if logs directory exists
            if (!fs.existsSync(logDir)) {
                res.status(200).json({ logs: [] });
                return;
            }
            
            // Read all files in the logs directory
            const files = fs.readdirSync(logDir);
            
            // Filter only .log files
            const logFiles = files.filter(file => file.endsWith('.log'));
            
            // Return the list of log files
            res.status(200).json({
                logs: logFiles
            });
        } catch (error) {
            console.error(`[MissionLogController] Error fetching mission logs: ${error}`);
            res.status(500).json({
                error: 'Failed to fetch mission logs'
            });
        }
    };

    /**
     * Get content of a specific mission log file
     * 
     * @param req Express Request object with filename parameter
     * @param res Express Response object
     * @returns Content of the requested log file
     */
    public getMissionLogContent = async (req: Request, res: Response): Promise<void> => {
        try {
            const { filename } = req.params;
            
            if (!filename) {
                res.status(400).json({
                    error: 'Filename parameter is required'
                });
                return;
            }

            // Validate filename to prevent directory traversal attacks
            if (filename.includes('..') || filename.includes('/') || filename.includes('\\')) {
                res.status(400).json({
                    error: 'Invalid filename'
                });
                return;
            }
            
            const findRootDir = () => {
                let currentDir = __dirname;
                while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                    const parentDir = path.dirname(currentDir);
                    if (parentDir === currentDir) {
                        return __dirname;
                    }
                    currentDir = parentDir;
                }
                return currentDir;
            };
        
            const serverRoot = findRootDir();
            const logFilePath = path.join(serverRoot, 'logs', filename);
            
            // Check if file exists
            if (!fs.existsSync(logFilePath)) {
                res.status(404).json({
                    error: 'Log file not found'
                });
                return;
            }
            
            // Read file content
            const content = fs.readFileSync(logFilePath, 'utf8');
            
            // Return the content
            res.status(200).json({
                filename,
                content
            });
        } catch (error) {
            console.error(`[MissionLogController] Error fetching log content: ${error}`);
            res.status(500).json({
                error: 'Failed to fetch log content'
            });
        }
    };

    /**
     * Delete a specific mission log file
     * 
     * @param req Express Request object with filename parameter
     * @param res Express Response object
     * @returns Success or error message
     */
    public deleteMissionLog = async (req: Request, res: Response): Promise<void> => {
        try {
            const { filename } = req.params;
            
            if (!filename) {
                res.status(400).json({
                    error: 'Filename parameter is required'
                });
                return;
            }

            // Validate filename to prevent directory traversal attacks
            if (filename.includes('..') || filename.includes('/') || filename.includes('\\')) {
                res.status(400).json({
                    error: 'Invalid filename'
                });
                return;
            }
            
            const findRootDir = () => {
                let currentDir = __dirname;
                while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                    const parentDir = path.dirname(currentDir);
                    if (parentDir === currentDir) {
                        return __dirname;
                    }
                    currentDir = parentDir;
                }
                return currentDir;
            };
        
            const serverRoot = findRootDir();
            const logFilePath = path.join(serverRoot, 'logs', filename);
            
            // Check if file exists
            if (!fs.existsSync(logFilePath)) {
                res.status(404).json({
                    error: 'Log file not found'
                });
                return;
            }
            
            // Delete the file
            fs.unlinkSync(logFilePath);
            
            // Return success message
            res.status(200).json({
                message: `Successfully deleted ${filename}`
            });
        } catch (error) {
            console.error(`[MissionLogController] Error deleting log file: ${error}`);
            res.status(500).json({
                error: 'Failed to delete log file'
            });
        }
    };
    public getLogByMissionId = async (req: Request, res: Response): Promise<void> => {
        try {
            const { missionId } = req.params;
    
            if (!missionId) {
                res.status(400).json({ error: 'Mission ID is required' });
            }
    
            const { ObjectId } = require('mongodb');
            const { connectToDatabase } = require('@app/mangodb/mission');
    
            const db = await connectToDatabase();
            const mission = await db.collection('missions').findOne({ _id: new ObjectId(missionId) });
    
            if (!mission || !mission.logFilename) {
                res.status(404).json({ error: 'No log associated with this mission' });
            }
    
            const findRootDir = () => {
                let currentDir = __dirname;
                while (!fs.existsSync(path.join(currentDir, 'package.json'))) {
                    const parentDir = path.dirname(currentDir);
                    if (parentDir === currentDir) return __dirname;
                    currentDir = parentDir;
                }
                return currentDir;
            };
    
            const serverRoot = findRootDir();
            const logFilePath = path.join(serverRoot, 'logs', mission.logFilename);
    
            if (!fs.existsSync(logFilePath)) {
                res.status(404).json({ error: 'Log file not found on disk' });
            }
    
            const content = fs.readFileSync(logFilePath, 'utf-8');
    
            res.status(200).json({
                filename: mission.logFilename,
                content
            });
        } catch (error) {
            console.error(`[MissionLogController] Error fetching log for mission:`, error);
            res.status(500).json({ error: 'Internal server error' });
        }
    };
    
    
}