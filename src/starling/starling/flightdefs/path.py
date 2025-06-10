class Path:
    def __init__(self, rate):
        self.rate = rate

    def get_state(self, step):
        """
        Returns the state of the path at a given step.
        This method must be implemented by subclasses.
        :param step: The current step in the path.
        :return: A State object representing the drone's state at the given step.
        """
        raise NotImplementedError("Subclasses must implement this method")

    def set_params(self, *args, **kwargs):
        """
        Sets parameters for the path.
        This method must be implemented by subclasses.
        :param args: Positional arguments for path parameters.
        :param kwargs: Keyword arguments for path parameters.
        """
        raise NotImplementedError("Subclasses must implement this method")
    
    def get_rate(self):
        """
        Returns the rate of the path.
        :return: The rate of the path.
        """
        return self.rate
    
    def get_length(self):
        """
        Returns the length of the path.
        This method must be implemented by subclasses.
        :return: The length of the path.
        """
        raise NotImplementedError("Subclasses must implement this method")
    
    def get_num_steps(self):
        """
        Returns the number of steps in the path.
        This method must be implemented by subclasses.
        :return: The number of steps in the path.
        """
        raise NotImplementedError("Subclasses must implement this method")