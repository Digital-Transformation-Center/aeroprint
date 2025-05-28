The helix should be defined by the parameters (meters):
- **h**: the height of the scan
- **r**: the radius of the helix
- **g**: the helix *gap*, i.e. the vertical distance between two levels

We will define a static speed, which will be a function of the radius, and this will determine $\theta$. The time for one revolution will be linear to $\theta$: $\theta = \frac{2\pi t}{30r}$


$$x = r\cos(\theta)$$
$$y = r\sin(\theta)$$

Now, we want z to be a function of $\theta$, where every multiple of $2\pi$, z increases by g. Thus, $z = g\frac{\theta}{2\pi}$

x and y repeat at $k2\pi$, so z should increase by g as theta increases by $2\pi$


Now the total time is the time of revolution by height over g, $t_{\text{total}}=30rh/2g$