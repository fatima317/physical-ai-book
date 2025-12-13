/**
 * StorageService
 * Handles file-based storage for content persistence
 */

import fs from 'fs/promises';
import path from 'path';
import { fileURLToPath } from 'url';

// Get the current directory name
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

class StorageService {
  constructor(basePath = './data') {
    this.basePath = basePath;
    this.ensureBasePath();
  }

  /**
   * Ensures the base path exists
   */
  async ensureBasePath() {
    try {
      await fs.mkdir(this.basePath, { recursive: true });
    } catch (error) {
      console.error(`Failed to create base path ${this.basePath}:`, error.message);
      throw error;
    }
  }

  /**
   * Saves data to a file
   * @param {string} collection - Collection name (directory)
   * @param {string} id - Document ID (filename without extension)
   * @param {Object} data - Data to save
   * @returns {Promise<Object>} Saved data with metadata
   */
  async save(collection, id, data) {
    try {
      const collectionPath = path.join(this.basePath, collection);
      await fs.mkdir(collectionPath, { recursive: true });

      const filePath = path.join(collectionPath, `${id}.json`);

      const dataToSave = {
        ...data,
        id,
        updatedAt: new Date().toISOString(),
        createdAt: data.createdAt || new Date().toISOString()
      };

      await fs.writeFile(filePath, JSON.stringify(dataToSave, null, 2));
      return dataToSave;
    } catch (error) {
      console.error(`Failed to save ${collection}/${id}:`, error.message);
      throw error;
    }
  }

  /**
   * Loads data from a file
   * @param {string} collection - Collection name (directory)
   * @param {string} id - Document ID (filename without extension)
   * @returns {Promise<Object|null>} Loaded data or null if not found
   */
  async load(collection, id) {
    try {
      const filePath = path.join(this.basePath, collection, `${id}.json`);
      const data = await fs.readFile(filePath, 'utf8');
      return JSON.parse(data);
    } catch (error) {
      if (error.code === 'ENOENT') {
        return null; // File doesn't exist
      }
      console.error(`Failed to load ${collection}/${id}:`, error.message);
      throw error;
    }
  }

  /**
   * Deletes a file
   * @param {string} collection - Collection name (directory)
   * @param {string} id - Document ID (filename without extension)
   * @returns {Promise<boolean>} True if deleted successfully
   */
  async delete(collection, id) {
    try {
      const filePath = path.join(this.basePath, collection, `${id}.json`);
      await fs.unlink(filePath);
      return true;
    } catch (error) {
      if (error.code === 'ENOENT') {
        return false; // File doesn't exist
      }
      console.error(`Failed to delete ${collection}/${id}:`, error.message);
      throw error;
    }
  }

  /**
   * Lists all files in a collection
   * @param {string} collection - Collection name (directory)
   * @returns {Promise<Array<Object>>} Array of documents
   */
  async list(collection) {
    try {
      const collectionPath = path.join(this.basePath, collection);
      const files = await fs.readdir(collectionPath);

      const documents = [];
      for (const file of files) {
        if (file.endsWith('.json')) {
          const id = path.basename(file, '.json');
          const doc = await this.load(collection, id);
          if (doc) {
            documents.push(doc);
          }
        }
      }

      return documents;
    } catch (error) {
      if (error.code === 'ENOENT') {
        return []; // Collection directory doesn't exist
      }
      console.error(`Failed to list ${collection}:`, error.message);
      throw error;
    }
  }

  /**
   * Searches for documents in a collection based on a filter function
   * @param {string} collection - Collection name
   * @param {Function} filter - Filter function
   * @returns {Promise<Array<Object>>} Array of matching documents
   */
  async search(collection, filter) {
    const allDocs = await this.list(collection);
    return allDocs.filter(filter);
  }

  /**
   * Checks if a document exists
   * @param {string} collection - Collection name
   * @param {string} id - Document ID
   * @returns {Promise<boolean>} True if document exists
   */
  async exists(collection, id) {
    try {
      const filePath = path.join(this.basePath, collection, `${id}.json`);
      await fs.access(filePath);
      return true;
    } catch {
      return false;
    }
  }

  /**
   * Updates a document by loading, modifying, and saving it
   * @param {string} collection - Collection name
   * @param {string} id - Document ID
   * @param {Object} updates - Updates to apply
   * @returns {Promise<Object|null>} Updated document or null if not found
   */
  async update(collection, id, updates) {
    const existing = await this.load(collection, id);
    if (!existing) {
      return null;
    }

    const updated = {
      ...existing,
      ...updates,
      id: existing.id, // Preserve the original ID
      updatedAt: new Date().toISOString()
    };

    return await this.save(collection, id, updated);
  }

  /**
   * Gets the file path for a document
   * @param {string} collection - Collection name
   * @param {string} id - Document ID
   * @returns {string} File path
   */
  getFilePath(collection, id) {
    return path.join(this.basePath, collection, `${id}.json`);
  }

  /**
   * Gets storage statistics
   * @returns {Promise<Object>} Storage statistics
   */
  async getStats() {
    try {
      const collections = await fs.readdir(this.basePath);
      const stats = {
        collections: {},
        totalDocuments: 0,
        totalSize: 0
      };

      for (const collection of collections) {
        const collectionPath = path.join(this.basePath, collection);
        const stat = await fs.stat(collectionPath);

        if (stat.isDirectory()) {
          const files = await fs.readdir(collectionPath);
          const collectionStats = {
            documents: 0,
            size: 0
          };

          for (const file of files) {
            if (file.endsWith('.json')) {
              const filePath = path.join(collectionPath, file);
              const fileStat = await fs.stat(filePath);
              collectionStats.documents++;
              collectionStats.size += fileStat.size;
              stats.totalDocuments++;
              stats.totalSize += fileStat.size;
            }
          }

          stats.collections[collection] = collectionStats;
        }
      }

      return stats;
    } catch (error) {
      console.error('Failed to get storage stats:', error.message);
      throw error;
    }
  }

  /**
   * Backs up the entire storage directory
   * @param {string} backupPath - Path to backup to
   * @returns {Promise<boolean>} True if backup was successful
   */
  async backup(backupPath) {
    try {
      // This would implement backup functionality
      // For now, we'll just copy the entire directory
      const { exec } = await import('child_process');
      return new Promise((resolve, reject) => {
        exec(`cp -r "${this.basePath}" "${backupPath}"`, (error) => {
          if (error) {
            console.error('Backup failed:', error);
            reject(error);
          } else {
            resolve(true);
          }
        });
      });
    } catch (error) {
      console.error('Backup failed:', error.message);
      return false;
    }
  }

  /**
   * Restores from a backup
   * @param {string} backupPath - Path to restore from
   * @returns {Promise<boolean>} True if restore was successful
   */
  async restore(backupPath) {
    try {
      // This would implement restore functionality
      // For now, we'll just copy from backup to base path
      const { exec } = await import('child_process');
      return new Promise((resolve, reject) => {
        exec(`cp -r "${backupPath}" "${this.basePath}"`, (error) => {
          if (error) {
            console.error('Restore failed:', error);
            reject(error);
          } else {
            resolve(true);
          }
        });
      });
    } catch (error) {
      console.error('Restore failed:', error.message);
      return false;
    }
  }
}

export { StorageService };