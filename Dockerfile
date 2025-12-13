# Use Node.js 18 LTS as the base image
FROM node:18-alpine

# Set working directory
WORKDIR /app

# Copy package files
COPY package*.json ./

# Install dependencies
RUN npm ci --only=production

# Copy the rest of the application code
COPY . .

# Expose port for the application
EXPOSE 3000

# Build the Docusaurus site
RUN npm run build

# Command to run the application
CMD ["npm", "run", "serve"]