# Approximating Euclidean Translation to Hyperbolic Geometry using Inverse Geometry in Pointcaré Disc

## Repo Details

### Numerical Analysis of the algorithm

`GeometricAnalysis.java` contains an numerical implementation with 128 bits precision of the algorithm.

Compile and run it with following commands (Windows PowerShell environment)
```shell
javac -cp .\Quadruple-1.2.0.jar .\GeometricAnalysis.java
java -cp ".;.\Quadruple-1.2.0.jar" GeometricAnalysis
```

`Quadruple-1.2.0.jar` was provided from https://github.com/m-vokhm/Quadruple

Performance of the algorithm. All precisions are able to perform zero results or could it be null values
which are messing around under the hood? Anyway by increasing the precision decreases the error significantly
which makes it suspiciously to be an exact solution.

### Symbolic Analysis of the algorithm

A geogebra activity is found https://www.geogebra.org/m/axkwdryv (the colinear case is not considered)

`geometry.wxmx` is a [Maxima](https://maxima.sourceforge.io/) representation of the algorithm (not completed yet).

An introduction to Hyperbolic constructions is found at this site: https://www.malinc.se/noneuclidean/en/index.php


### To-do list

- [ ] Investigate single-precision's max-error
- [ ] Investigate single-precision's NaN values
- [ ] See if one can optimize the code with help of maxima by reducing floating-point operations
- [ ] Investigate $Z_i$ is set to $O$


## Abstract
In geometry, many has study the properties of the pointcaré disc, which is unit-disc excluding the boundary. It is the foundation for the hyperbolic geometry which enabling more tesselation configurations, and it might be of interest of gamedevelopers to use such a tesselation for the gaming world.  However the hyperbolic geometry is non-euclidean which means the translation by a mouse-pointer, which coordinates is given by euclidean coordinates for a point is non-trivial, since it has to be transformed into circles to be used when transforming the coordinates in the hyperbolic geometric space.

There have been several work covering Möbius transformation in poincaré disk model, but no work was found about transferring euclidean transformation into hyperbolic transformations.

In this paper a geometric construction is approximating the euclidean translation transformation in the hyperbolic space is presented.

## Literature Survey

The take away from the literature is that two reflections can create a translation, and in hyperbolic geometry one use reflections to *transfer* points in the geometric constructions [https://en.wikipedia.org/wiki/Inversive_geometry#In_higher_dimensions]. All transformations in pointcaré disk model, or inside the unit disc are Möbius transformations [https://en.wikipedia.org/wiki/Hyperbolic_geometry].

## Definitions

In **Appendix A** one may found the equations used for each function definition in the list below.

$$
\begin{align}
O :=& \text{ The Origin $(0,0,0)$}\\
Z :=& \text{ Up-vector $(0,0,1)$} \\
C(P, Q) :=& \text{ Circle through point $Q$ with centre $P$} \\
C_U := & \text{ Unit circle, $C(O,(1,0,0))$}\\
C(A, B, C) :=& \text{ Circle through point $A$, $B$ and $C$} \\
S(A,B):=& \text{ Euclidean Line/Segment between point $A$ and $B$}\\
I_i(C_1, C_2) \rightarrow \set{I_1, I_2} :=& \text{ Intersection of two euclidean circles, $i \in \set{1,2}$ denotes which intersection} \\
I(A_1, A_2, B_1, B_2) :=& \text{ Intersection of two lines}\\
sgn(x) \rightarrow \set{1,-1} :=& \text{ $-1$ when $x\lt 0$ otherwise $1$}\\
I_i(A_1, A_2, C) \rightarrow \set{I_1, I_2} :=& \text{ Intersection of a line and a circle }\\
R(P,C) \rightarrow U :=& \text{ Reflection of point $P$ about circle $C$ returning point $U$}\\
R(P,S) \rightarrow U :=& \text{ Reflection of point $P$ about line $S$ returning point $U$}\\
C_H(P,Q) \rightarrow C(\cdot, \cdot) :=& \text{ Hyperbolic circle with its centre at $P$ and goes through point $Q$} \\
S_H(P,Q) \rightarrow \set{C(\cdot, \cdot), S(P,Q)}:=& \text{ Hyperbolic line segment in pointcaré disc between point $P$ and $Q$} \\
M(P, Q) \rightarrow \set{C(\cdot,\cdot), S(P,Q)}  :=& \text{ Circle or line segment to perform reflection about} \\
M_D(X,P,Q, \gamma) \rightarrow U :=&\text{ Dilation/move/rotation transform in hyperbolic space. } \\
M_D^{-1}(X,P,Q, \gamma) \rightarrow U :=&~M_D(X,Q,P, \gamma)\text{ Inverse dilation/move/rotation transform in hyperbolic space. } 
\end{align}
$$


$$
\begin{gather}
M_D(X,P,Q, \gamma) \rightarrow U := \notag \\
R\left(\\
R\left(X, M\left(P,\frac{1}{2}\left(P + Q\right)+ \gamma \frac{(P-Q)}{\lVert P-Q\rVert} \times Z\right) \right), M\left(\frac{1}{2}(P + Q) + \gamma \frac{(P-Q)}{\lVert P-Q\rVert} \times Z,Q\right)\right)
\end{gather}
$$

## Theorem
There exists a geometric construction which transform the euclidean translation from $A$ to $B$ in pointcaré disc such that one can construct two reflection circles in hyperbolic geometry which translates $A$ to $B$ with an error of $\varepsilon$ when it is reflected about those circles. The hyperbolic center of the circles are $\xi$ and $\frac{\xi}{2}$. 

$$
\begin{equation}
M_D(A,O,\xi,0) = B + \varepsilon
\end{equation}
$$

$$
\begin{gather}
Z_0 = M_D(O, A, B, 0)\\
Z_1 = M_D(O, A, B, \gamma_1)\\
Z_2 = M_D(O, A, B, \gamma_2)\\
\notag \\
\lVert A \rVert \lt 1.0, \quad\lVert B \rVert \lt 1.0,\\
 \gamma_1 \neq 0, \quad \gamma_2 \neq 0,\quad \gamma_1 \neq \gamma_2 \\
Z_i = O,  \text{ when } M_D(\cdot, \cdot, \cdot, \cdot) \rightarrow S(\cdot, \cdot) \\
\notag \\
\xi = \begin{cases}
O, \quad Z_i \propto Z_j \quad i \neq j \quad i,j \in \set{0,1,2} \\
O, \quad \lVert C_{center}(Z_0, Z_1, Z_2) \rVert \gt 1 \\
 C_{center}\left(Z_0,Z_1,Z_2\right) - C_{radius}(Z_0,Z_1,Z_2)\frac{A}{\lVert A\rVert}, \text{otherwise}.\\
 \end{cases}
\end{gather}
$$

The function $M_D$ hides the fact that there are two reflection circles in operation, for further details study the defintion of it.

The theorem says that there exists an circle which is created by transforming the origo with the translation operation between $A$ and $B$ with different paths. It is some kind of dual space. The different path creates points on the circle.

## Implementation Details

The algorithm has been implemented in Java. The floating precisions used are 32 (single), 64 (double) and 128 + 32 (quadruple* by https://github.com/m-vokhm/Quadruple). Points are considered colinear if $|A_xB_y - A_yB_x|\lt \tau$. See table for thresholds of $\tau$ for different precisions.

$$
\begin{array}{l|c}
\text{Precision} & \tau \\
\hline
\text{single} & 10^{-6}\\
\text{double} & 10^{-12}\\
\text{quadruple}^* & 10^{-30}
\end{array}
$$

The original implementation was done for a game-engine which a left-handed coordinate system where the up-vector $Z=(0,1,0)$ was used. This configuration remains in implementation, wheras the equations considering the the $XY$-plane.

$\gamma_1$ and $\gamma_2$ was set to 0.001 respectively -0.001 to keep some numerical stability to the problem.

Points are generated with quadruple* precision and then downcast to double and single precision. For colinear points, a direction first sampled and then scaled with two different sampled values in range $[0,1]$.

### Simplifications


#### Hyperbolic Circle

The function $C_H(P,Q)$ was simplified using Maxima to become:

$$
\begin{equation}
C_H(P,Q) = C(I(O,P,Q, Q + Z\times(Q - C(P,Q, R(Q, C_U)))), Q) = C\left(P\cdot\frac{(Q \cdot Q - 1)}{(2 P\cdot Q - P\cdot P - 1)}, Q\right)
\end{equation}
$$

For the colinear case:

$$
\begin{equation}
C_H(P,\delta P) = C\left(P\cdot\frac{\delta^2 P \cdot P - 1}{2 \delta P\cdot P - P\cdot P - 1}, \delta P\right)
\end{equation}
$$

it is necessary to investigate the denominator $2 \delta P\cdot P - P\cdot P - 1 = 0$ to avoid an 0 denominator. Solving for $\delta$ gives following:

$$
\begin{equation}
\delta = \frac{P_y^2+P_x^2+1}{2 (P_y^2 +  P_x^2)} =\frac{\lVert P \rVert^2 + 1}{2 \lVert P \rVert^2}
\end{equation}
$$

Since all points in the pointcaré disk model lies within the unit-circle, it is possible to show that $\delta$ will only be reached iff the point is outside the pointcaré disk:

$$
\begin{gather}
\lVert \delta \cdot P \rVert^2 \geq 1 \\
\left(\frac{\lVert P \rVert^2 + 1}{2 \lVert P \rVert^2}\right)^2\lVert P \rVert^2 \geq 1 \\
\frac{(\lVert P \rVert^2 + 1)^2}{4 \lVert P \rVert^2} \geq 1 \\
(\lVert P \rVert^2 + 1)^2 \geq 4 \lVert P \rVert^2 \\
x = \lVert P \rVert^2, \quad 0 \leq x < 1 \implies \\
(x + 1)^2 \gt 4x 
\end{gather}
$$

Therefore it is possible to just simplify the expression to:

$$
\begin{equation}
C_H(P,Q)=C\left(P\cdot\frac{Q \cdot Q - 1}{2 P\cdot Q - P\cdot P - 1}, Q\right)
\end{equation}
$$

<!-- #### Circle-Circle Intersection

An alternative computation method of the circle circle intersection to attempt to avoid a denominator in the squareroot, it is formulated from this expression:

$$
\begin{gather}
I_{1,x} = \frac{\left( {G_y} - {F_y}\right)  \sqrt{ - {{{R_2}}^{2}} + 2 {R_1} {R_2} - {{{R_1}}^{2}} + {{{G_y}}^{2}} - 2 {F_y} {G_y} + {{{G_x}}^{2}} - 2 {F_x} {G_x} + {{{F_y}}^{2}} + {{{F_x}}^{2}}} \sqrt{{{{R_2}}^{2}} + 2 {R_1} {R_2} + {{{R_1}}^{2}} - {{{G_y}}^{2}} + 2 {F_y} {G_y} - {{{G_x}}^{2}} + 2 {F_x} {G_x} - {{{F_y}}^{2}} - {{{F_x}}^{2}}} + \left( {F_x} - {G_x}\right)  {{{R_2}}^{2}} + \left( {G_x} - {F_x}\right)  {{{R_1}}^{2}} + \left( {G_x} + {F_x}\right)  {{{G_y}}^{2}} + \left(  - \left( 2 {F_y} {G_x}\right)  - 2 {F_x} {F_y}\right)  {G_y} + {{{G_x}}^{3}} - {F_x} {{{G_x}}^{2}} + \left( {{{F_y}}^{2}} - {{{F_x}}^{2}}\right)  {G_x} + {F_x} {{{F_y}}^{2}} + {{{F_x}}^{3}}}{2 {{{G_y}}^{2}} - 4 {F_y} {G_y} + 2 {{{G_x}}^{2}} - 4 {F_x} {G_x} + 2 {{{F_y}}^{2}} + 2 {{{F_x}}^{2}}}\\
I_{1,y} = -\left( \frac{\left( {G_x} - {F_x}\right)  \sqrt{ - {{{R_2}}^{2}} + 2 {R_1} {R_2} - {{{R_1}}^{2}} + {{{G_y}}^{2}} - 2 {F_y} {G_y} + {{{G_x}}^{2}} - 2 {F_x} {G_x} + {{{F_y}}^{2}} + {{{F_x}}^{2}}} \sqrt{{{{R_2}}^{2}} + 2 {R_1} {R_2} + {{{R_1}}^{2}} - {{{G_y}}^{2}} + 2 {F_y} {G_y} - {{{G_x}}^{2}} + 2 {F_x} {G_x} - {{{F_y}}^{2}} - {{{F_x}}^{2}}} + \left( {G_y} - {F_y}\right)  {{{R_2}}^{2}} + \left( {F_y} - {G_y}\right)  {{{R_1}}^{2}} - {{{G_y}}^{3}} + {F_y} {{{G_y}}^{2}} + \left(  - {{{G_x}}^{2}} + 2 {F_x} {G_x} + {{{F_y}}^{2}} - {{{F_x}}^{2}}\right)  {G_y} - {F_y} {{{G_x}}^{2}} + 2 {F_x} {F_y} {G_x} - {{{F_y}}^{3}} - {{{F_x}}^{2}} {F_y}}{2 {{{G_y}}^{2}} - 4 {F_y} {G_y} + 2 {{{G_x}}^{2}} - 4 {F_x} {G_x} + 2 {{{F_y}}^{2}} + 2 {{{F_x}}^{2}}}\right)
\end{gather}
$$

with the circles $C(F, R_1 e_x + F)$ and $C(G, R_1 e_x + G)$ where $e_x$ is the unit vector of the $x$-axis by making substition as following.

$$
\begin{gather}
p_1 : -(R_2 - R_1)^2 \\
p_2 : (R_2 + R_1)^2 \\ 
p_3 : (F_y-G_y)^2+(F_x-G_x)^2
\end{gather}
$$ -->

## Results

*Simplified* means using the revised simplification of $C_H$ 
<!-- and *intersection* means a revised circle-circle intersection
algorithm. -->


Errors for 100000 random points transformations $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 2.972 \cdot 10^{-04} & 1.888 \cdot 10^{+00} & 854 & 0\\
\text{double} & 5.607 \cdot 10^{-12} & 3.448 \cdot 10^{-07} & 889&\\
\text{quadruple}^* & 1.175 \cdot 10^{-34} & 8.686 \cdot 10^{-30} & 10386&\\
\end{array}
$$


Errors for 100000 random points transformations simplified $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 3.793 \cdot 10^{-06} & 7.303 \cdot 10^{-02} & 2043 & 0\\
\text{double} & 9.979 \cdot 10^{-15} & 2.653 \cdot 10^{-10} & 2088&\\
\text{quadruple}^* & 1.286 \cdot 10^{-37} & 2.331 \cdot 10^{-33} & 13581&\\
\end{array}
$$


<!-- Errors for 100000 random points transformations intersection $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 2.974 \cdot 10^{-04} & 1.888 \cdot 10^{+00} & 875 & 0\\
\text{double} & 5.603 \cdot 10^{-12} & 3.447 \cdot 10^{-07} & 853&\\
\text{quadruple}^* & 1.175 \cdot 10^{-34} & 8.686 \cdot 10^{-30} & 10523&\\
\end{array}
$$


Errors for 100000 random points transformations intersection and simplified $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 6.295 \cdot 10^{-06} & 1.917 \cdot 10^{-01} & 2131 & 0\\
\text{double} & 1.186 \cdot 10^{-14} & 2.580 \cdot 10^{-10} & 2010&\\
\text{quadruple}^* & 1.246 \cdot 10^{-37} & 2.383 \cdot 10^{-33} & 13769&\\
\end{array}
$$ -->


Errors for 100000 random points transformations $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 7.500 \cdot 10^{-02} & 1.981 \cdot 10^{+00} & 0 & 0\\
\text{double} & 6.599 \cdot 10^{-10} & 1.018 \cdot 10^{-05} & 0&\\
\text{quadruple}^* & 1.467 \cdot 10^{-32} & 5.504 \cdot 10^{-28} & 20&\\
\end{array}
$$


Errors for 100000 random points transformations simplified $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 7.383 \cdot 10^{-02} & 1.986 \cdot 10^{+00} & 0 & 0\\
\text{double} & 9.531 \cdot 10^{-10} & 2.488 \cdot 10^{-05} & 0&\\
\text{quadruple}^* & 9.952 \cdot 10^{-33} & 2.896 \cdot 10^{-28} & 35&\\
\end{array}
$$


<!-- Errors for 100000 random points transformations intersection $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 7.546 \cdot 10^{-02} & 1.986 \cdot 10^{+00} & 1 & 0\\
\text{double} & 7.860 \cdot 10^{-10} & 1.542 \cdot 10^{-05} & 0&\\
\text{quadruple}^* & 1.560 \cdot 10^{-32} & 5.865 \cdot 10^{-28} & 13&\\
\end{array}
$$


Errors for 100000 random points transformations intersection and simplified $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 7.360 \cdot 10^{-02} & 1.986 \cdot 10^{+00} & 0 & 0\\
\text{double} & 7.543 \cdot 10^{-10} & 1.105 \cdot 10^{-05} & 0&\\
\text{quadruple}^* & 9.959 \cdot 10^{-33} & 1.226 \cdot 10^{-28} & 24&\\
\end{array}
$$ -->


Errors for 100000 colinear points transformations $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 4.004 \cdot 10^{-06} & 1.148 \cdot 10^{-01} & 3274 & 0\\
\text{double} & 6.422 \cdot 10^{-15} & 1.201 \cdot 10^{-10} & 3342&\\
\text{quadruple}^* & 1.365 \cdot 10^{-37} & 5.652 \cdot 10^{-33} & 3814&\\
\end{array}
$$


Errors for 100000 colinear points transformations simplified $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 4.358 \cdot 10^{-06} & 1.376 \cdot 10^{-01} & 2584 & 0\\
\text{double} & 1.070 \cdot 10^{-14} & 5.033 \cdot 10^{-10} & 2590&\\
\text{quadruple}^* & 1.013 \cdot 10^{-37} & 4.507 \cdot 10^{-33} & 2809&\\
\end{array}
$$


<!-- Errors for 100000 colinear points transformations intersection $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 3.066 \cdot 10^{-06} & 1.107 \cdot 10^{-01} & 3532 & 0\\
\text{double} & 5.112 \cdot 10^{-15} & 1.135 \cdot 10^{-10} & 3616&\\
\text{quadruple}^* & 1.254 \cdot 10^{-37} & 6.045 \cdot 10^{-33} & 4024&\\
\end{array}
$$


Errors for 100000 colinear points transformations intersection and simplified $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 3.751 \cdot 10^{-06} & 8.000 \cdot 10^{-02} & 2751 & 0\\
\text{double} & 4.223 \cdot 10^{-15} & 6.417 \cdot 10^{-11} & 2772&\\
\text{quadruple}^* & 7.195 \cdot 10^{-38} & 1.816 \cdot 10^{-33} & 3003&\\
\end{array}
$$ -->


Errors for 100000 colinear points transformations $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 1.508 \cdot 10^{-02} & 1.890 \cdot 10^{+00} & 0 & 0\\
\text{double} & 1.162 \cdot 10^{-08} & 7.530 \cdot 10^{-04} & 0&\\
\text{quadruple}^* & 2.286 \cdot 10^{-31} & 1.473 \cdot 10^{-26} & 1&\\
\end{array}
$$


Errors for 100000 colinear points transformations simplified $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 6.258 \cdot 10^{-03} & 1.991 \cdot 10^{+00} & 5 & 0\\
\text{double} & 8.697 \cdot 10^{-12} & 1.628 \cdot 10^{-07} & 5&\\
\text{quadruple}^* & 1.015 \cdot 10^{-34} & 1.277 \cdot 10^{-30} & 5&\\
\end{array}
$$


<!-- Errors for 100000 colinear points transformations intersection $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 1.506 \cdot 10^{-02} & 1.803 \cdot 10^{+00} & 0 & 0\\
\text{double} & 1.161 \cdot 10^{-08} & 7.530 \cdot 10^{-04} & 0&\\
\text{quadruple}^* & 2.285 \cdot 10^{-31} & 1.473 \cdot 10^{-26} & 1&\\
\end{array}
$$


Errors for 100000 colinear points transformations intersection and simplified $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 6.279 \cdot 10^{-03} & 1.998 \cdot 10^{+00} & 7 & 0\\
\text{double} & 1.031 \cdot 10^{-11} & 1.351 \cdot 10^{-07} & 2&\\
\text{quadruple}^* & 1.032 \cdot 10^{-34} & 1.696 \cdot 10^{-30} & 1&\\
\end{array}
$$ -->




Higher precisions decreases the error, it can be seen for all cases. The precision is dropped when changing to transforms on the form $M_D(A,O,\xi,0)$, which could be explained that there are so many heavy numerical operations which affects the precision. By simplifying the equations the error cut by half its size.  Since higher precision was not investigated, is is from this shortage of data possible to make following assumption:

$$
\begin{equation}
\lim_{precision \rightarrow \infty} \lVert \varepsilon \rVert \rightarrow 0.
\end{equation}
$$

The number of zeros column and the NaN column, those numbers are excluded from the averaging. The averaging is of 100000 points, so that the total number of samples is then 100000 plus the zero and NaN count.

The simplified version does produce more zero-value results which support the hypothesis of being a numerical hard problem.

The intersection version does not make a consistent improvement. There is not so much simplification as it is in simplified version, so it is hard to say if it performs better or worse. It has therefore been let out from the result table for readability, but can be found in the source of this document.

The high error in the single precision case have not been investigated, and is to be considered a todo item.

## Discussion

The literature survey was limited since it would be too much effort to read up on a complete new field and the science related to the Möbius transformation. So there might have been algorithms which cover the same problem but was not found. The literature tells that all transformation inside the pointcaré disk model are Möbius transformations, so it should be possible to express the theorem using Möbius transformations.

With higher precision the results approaches zero. That makes the argument that the geometric construction is exact, even if the author have not been able to prove it theoretical. The most important part is to considering the decrease of magnitude of the error since they may vary between executions.

The simplification confirms that it is a numerical hard problem, and more simplicifications are required in order to proove it numerical.

There might be some more improvement possible to the theorem since the zero handling of $Z_i$ and $\xi$ could be refined.

The determination of $\gamma_1$ and $\gamma_2$ could be investigated more closely, perhaps a grid-search within a small range of their chosen values could improve the calculations.



## Conclusion

An algorithm for interpreting and euclidean translation into hyperbolic or inversive geometry has been presented. It performs well, and even better with higher precision. Since the higher precision is achieving a smaller error and it might be considered as exact, but no proof has been provided but is welcome in any way,counter-proof or proof.


# Appendix A

Here follows definitions of all functions

---

$$
C(P,Q) = \lVert x-P\rVert^2 = \lVert Q - P\rVert^2
$$

---

$$
C(A,B,C) = C\left(\left(
	\frac{A \cdot A \cdot (B_y - C_y) +
	B \cdot B  \cdot (C_y - A_y) +
	C \cdot C  \cdot (A_y - B_y)}{
	2 (A_x \cdot (B_y - C_y) + B_x \cdot (C_y - A_y) + C_x \cdot (A_y - B_y))
	}, 
	\frac{A \cdot A \cdot (B_x - C_x) +
	B \cdot B  \cdot (C_x - A_x) +
	C \cdot C  \cdot (A_x - B_x)}{
	2 (A_x \cdot (B_y - C_y) + B_x \cdot (C_y - A_y) + C_x \cdot (A_y - B_y))
	}
					 \right),A\right)
$$

---


$$
I_i(C_1, C_2) , i \in \set{1,2} \quad \text{$i$ denotes which intersection}\\
$$

$$
I_1 = \frac{C_{1,center} - C_{2,center}}{\lVert C_{1,center} - C_{2,center}\rVert} + C_{2, center} + \sqrt{C_{2,radius}^2 -\left( C_{2,radius}^2 - C_{1, radius}^2 + \lVert C_{1,center} - C_{2, center}\rVert^2 \right)} \left( Z\times \frac{C_{1,center} - C_{2,center}}{\lVert C_{1,center} - C_{2,center}\rVert}\right)\\
$$

$$
I_2 = \frac{C_{1,center} - C_{2,center}}{\lVert C_{1,center} - C_{2,center}\rVert} + C_{2, center} - \sqrt{C_{2,radius}^2 -\left( C_{2,radius}^2 - C_{1, radius}^2 + \lVert C_{1,center} - C_{2, center}\rVert^2 \right)} \left( Z\times \frac{C_{1,center} - C_{2,center}}{\lVert C_{1,center} - C_{2,center}\rVert}\right)
$$

---

$$
I(A_1, A_2, B_1, B_2)  = A_1 + \frac{(A_2-A_1)}{\lVert (A_2 - A_1) \times (B_2 - B_1)\rVert^2}\cdot ~~ (B_1 - A_1)\times(B_2 - B_1) ~~ \cdot ~~ (A_2 - A_1) \times (B_2 - B_1) 
$$

---

$$
% D = (A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x\\
% d = C_{radius}^2(A_2 - A_1)(A_2-A_1) - \left((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x\right)^2 \\
% s = \sqrt{(C_{radius}^2(A_2 - A_1)(A_2-A_1) - \left((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x\right)^2)}\\
I_1(A_1, A_2, C) = C_{center} + \frac{1}{(A_2 - A_1)(A_2-A_1)}
 \begin{bmatrix}
((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x)(A_2 - A_1)_y + sgn((A_2-A_1)_y)(A_2-A_1)_x\sqrt{(C_{radius}^2(A_2 - A_1)(A_2-A_1) - \left((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x\right)^2)}\\
((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x)(A_2 - A_1)_x  + |(A_2 - A_1)_y|\sqrt{(C_{radius}^2(A_2 - A_1)(A_2-A_1) - \left((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x\right)^2)} \\
0\\
\end{bmatrix}
$$

$$
I_2(A_1, A_2, C) = C_{center} + \frac{1}{(A_2 - A_1)(A_2-A_1)}
 \begin{bmatrix}
((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x)(A_2 - A_1)_y - sgn((A_2-A_1)_y)(A_2-A_1)_x\sqrt{(C_{radius}^2(A_2 - A_1)(A_2-A_1) - \left((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x\right)^2)}\\
((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x)(A_2 - A_1)_x  - |(A_2 - A_1)_y|\sqrt{(C_{radius}^2(A_2 - A_1)(A_2-A_1) - \left((A_1 - C_{center})_x (C_{center} + A_2)_y - (A_1 - C_{center})_y (C_{center} + A_2)_x\right)^2)} \\
0\\
\end{bmatrix}
\\\\
$$

---

$$
R(P,C) = \left(\frac{C_{radius}^2}{\lVert P-C_{center}\rVert^2}\left(P-C_{center}\right) + C_{center}\right)
$$

---

$$
R(P,S) = P - 2\left( (P - S_0) - \left((P - S_0)\cdot\frac{S_1 - S_0}{\lVert S_1 -S_0\rVert}\right)\cdot\frac{S_1 - S_0}{\lVert S_1 -S_0\rVert}\right)
$$

---

$$
C_H(P,Q) =\begin{cases}
C(0, Q), ~ P = O \\
C\left(\frac{1}{2}\left(Q + R\left(Q, C\left(\frac{1}{2}\left(P + R(P,C_U)\right), P \right)\right)\right), Q\right), ~ P \propto Q\\
C(I(O,P,Q, Q + Z\times(Q - C(P,Q, R(Q, C_U)))), Q), ~ P \not\propto  Q
\end{cases}
$$

---

$$
S_H(P,Q) = \begin{cases}
C(P, Q, R(P, C_U)), P \not\propto Q \\
S(P,Q), P \propto Q
\end{cases}
$$

---

$$
M(P,Q) = S_H(I_1(C_H(P,Q), C_H(Q,P)),I_2(C_H(P,Q), C_H(Q,P)))
$$

---

$$
\begin{gather*}
M_D(X,P,Q, \gamma) \rightarrow U :=\\
R\left(\\
R\left(X, M\left(P,\frac{1}{2}\left(P + Q\right)+ \gamma \frac{(P-Q)}{\lVert P-Q\rVert} \times Z\right) \right), M\left(\frac{1}{2}(P + Q) + \gamma \frac{(P-Q)}{\lVert P-Q\rVert} \times Z,Q\right)\right)
\end{gather*}
$$