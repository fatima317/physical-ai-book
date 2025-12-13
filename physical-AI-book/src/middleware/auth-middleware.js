/**
 * Authentication and Authorization Middleware
 * Provides authentication and authorization functionality for the API
 */

import jwt from 'jsonwebtoken';

class AuthMiddleware {
  constructor(secret = process.env.JWT_SECRET || 'default-secret-for-development') {
    this.secret = secret;
  }

  /**
   * Creates a JWT token for a user
   * @param {Object} user - User object
   * @returns {string} JWT token
   */
  createToken(user) {
    return jwt.sign(
      {
        id: user.id,
        email: user.email,
        role: user.role || 'user',
        permissions: user.permissions || []
      },
      this.secret,
      { expiresIn: '24h' }
    );
  }

  /**
   * Verifies a JWT token
   * @param {string} token - JWT token to verify
   * @returns {Object|null} Decoded token payload or null if invalid
   */
  verifyToken(token) {
    try {
      return jwt.verify(token, this.secret);
    } catch (error) {
      return null;
    }
  }

  /**
   * Middleware to authenticate user
   * @returns {Function} Express middleware function
   */
  authenticate() {
    return (req, res, next) => {
      // Check for token in header, query parameter, or cookie
      let token = req.headers.authorization;

      if (token && token.startsWith('Bearer ')) {
        token = token.slice(7, token.length);
      } else if (req.query.token) {
        token = req.query.token;
      } else if (req.cookies && req.cookies.token) {
        token = req.cookies.token;
      }

      if (!token) {
        return res.status(401).json({
          error: 'Access denied. No token provided.',
          code: 'NO_TOKEN'
        });
      }

      const decoded = this.verifyToken(token);
      if (!decoded) {
        return res.status(401).json({
          error: 'Invalid token.',
          code: 'INVALID_TOKEN'
        });
      }

      // Attach user info to request
      req.user = decoded;
      next();
    };
  }

  /**
   * Middleware to authorize user by role
   * @param {Array<string>} allowedRoles - Array of allowed roles
   * @returns {Function} Express middleware function
   */
  authorize(allowedRoles = ['user', 'admin']) {
    return (req, res, next) => {
      if (!req.user) {
        return res.status(401).json({
          error: 'Authentication required.',
          code: 'AUTH_REQUIRED'
        });
      }

      const userRole = req.user.role;
      if (!allowedRoles.includes(userRole)) {
        return res.status(403).json({
          error: 'Access denied. Insufficient permissions.',
          code: 'INSUFFICIENT_PERMISSIONS'
        });
      }

      next();
    };
  }

  /**
   * Middleware to authorize user by specific permissions
   * @param {Array<string>} requiredPermissions - Array of required permissions
   * @returns {Function} Express middleware function
   */
  requirePermissions(requiredPermissions = []) {
    return (req, res, next) => {
      if (!req.user) {
        return res.status(401).json({
          error: 'Authentication required.',
          code: 'AUTH_REQUIRED'
        });
      }

      const userPermissions = req.user.permissions || [];
      const hasPermissions = requiredPermissions.every(permission =>
        userPermissions.includes(permission)
      );

      if (!hasPermissions) {
        return res.status(403).json({
          error: 'Access denied. Missing required permissions.',
          code: 'MISSING_PERMISSIONS',
          required: requiredPermissions,
          user: userPermissions
        });
      }

      next();
    };
  }

  /**
   * Middleware to check if the user owns the resource
   * @param {string} resourceParam - Parameter name for the resource owner ID
   * @returns {Function} Express middleware function
   */
  ownsResource(resourceParam = 'userId') {
    return (req, res, next) => {
      if (!req.user) {
        return res.status(401).json({
          error: 'Authentication required.',
          code: 'AUTH_REQUIRED'
        });
      }

      const resourceId = req.params[resourceParam] || req.query[resourceParam] || req.body[resourceParam];
      const userId = req.user.id;

      if (resourceId && userId !== resourceId) {
        return res.status(403).json({
          error: 'Access denied. You do not own this resource.',
          code: 'RESOURCE_OWNERSHIP_REQUIRED'
        });
      }

      next();
    };
  }

  /**
   * Middleware to check if user is admin
   * @returns {Function} Express middleware function
   */
  adminOnly() {
    return this.authorize(['admin']);
  }

  /**
   * Middleware to check if user is teacher/educator
   * @returns {Function} Express middleware function
   */
  educatorOnly() {
    return this.authorize(['admin', 'educator', 'teacher']);
  }

  /**
   * Middleware to check if user is student/learner
   * @returns {Function} Express middleware function
   */
  learnerOnly() {
    return this.authorize(['admin', 'educator', 'teacher', 'student', 'learner']);
  }

  /**
   * Middleware to handle optional authentication
   * @returns {Function} Express middleware function
   */
  optionalAuth() {
    return (req, res, next) => {
      // Check for token in header, query parameter, or cookie
      let token = req.headers.authorization;

      if (token && token.startsWith('Bearer ')) {
        token = token.slice(7, token.length);
      } else if (req.query.token) {
        token = req.query.token;
      } else if (req.cookies && req.cookies.token) {
        token = req.cookies.token;
      }

      if (token) {
        const decoded = this.verifyToken(token);
        if (decoded) {
          req.user = decoded;
        }
      }

      next();
    };
  }

  /**
   * Generates a password hash (simplified for this implementation)
   * @param {string} password - Plain text password
   * @returns {string} Hashed password
   */
  hashPassword(password) {
    // In a real implementation, you would use bcrypt or similar
    // This is a simplified version for demonstration
    const crypto = require('crypto');
    return crypto.createHash('sha256').update(password + this.secret).digest('hex');
  }

  /**
   * Compares a password with a hash
   * @param {string} password - Plain text password
   * @param {string} hash - Hashed password
   * @returns {boolean} True if password matches hash
   */
  comparePassword(password, hash) {
    // In a real implementation, you would use bcrypt or similar
    const crypto = require('crypto');
    const hashedPassword = crypto.createHash('sha256').update(password + this.secret).digest('hex');
    return hashedPassword === hash;
  }
}

// Export a singleton instance
const authMiddleware = new AuthMiddleware();
export { authMiddleware, AuthMiddleware };