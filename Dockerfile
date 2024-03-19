# Use the official Ubuntu 20.04 base image
FROM ubuntu:20.04

# Update package lists and install any necessary packages
RUN apt-get update && apt-get install -y \
    # Add your desired packages here (e.g., curl, vim, etc.)
    curl \
    vim

# Set environment variables if needed
# ENV MY_ENV_VAR=value

# Start a shell when the container runs
CMD ["bash"]
