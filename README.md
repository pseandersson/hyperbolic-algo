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

The theorem says that there exists an circle which are created by transforming the origo with the translation operation between $A$ and $B$ with different paths. It is some kind of dual space. The different path creates points on the circle.

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

## Results


Errors for 100000 random points transformations $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 2.233 \cdot 10^{-04} & 1.607 \cdot 10^{+00} & 654 & 0\\
\text{double} & 6.719 \cdot 10^{-11} & 6.622 \cdot 10^{-06} & 714&\\
\text{quadruple}^* & 5.795 \cdot 10^{-33} & 5.790 \cdot 10^{-28} & 8083&\\
\end{array}
$$


Errors for 100000 random points transformations $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 7.382 \cdot 10^{-02} & 1.977 \cdot 10^{+00} & 0 & 0\\
\text{double} & 5.740 \cdot 10^{-10} & 7.253 \cdot 10^{-06} & 0&\\
\text{quadruple}^* & 1.384 \cdot 10^{-32} & 7.056 \cdot 10^{-28} & 7&\\
\end{array}
$$


Errors for 100000 colinear points transformations $M_D(A,A,B,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 3.326 \cdot 10^{-06} & 1.885 \cdot 10^{-01} & 3114 & 0\\
\text{double} & 3.402 \cdot 10^{-15} & 8.645 \cdot 10^{-11} & 3142&\\
\text{quadruple}^* & 8.550 \cdot 10^{-38} & 3.189 \cdot 10^{-33} & 3580&\\
\end{array}
$$


Errors for 100000 colinear points transformations $M_D(A,0,\xi,0)$:

$$
\begin{array}{lcccc}
\text{Precision} & \bar{\lVert\varepsilon\rVert} & \hat{\lVert\varepsilon\rVert} & 0 & \text{NaNs}\\
\hline
\text{single} & 1.696 \cdot 10^{-02} & 1.940 \cdot 10^{+00} & 2 & 0\\
\text{double} & 4.322 \cdot 10^{-08} & 4.282 \cdot 10^{-03} & 1&\\
\text{quadruple}^* & 1.861 \cdot 10^{-31} & 1.447 \cdot 10^{-26} & 1&\\
\end{array}
$$



Higher precisions decreases the error, it can be seen for all cases. The precision is dropped when changing to transforms on the form $M_D(A,O,\xi,0)$, which could be explained that there are so many heavy numerical operations which affects the precision. Since higher precision was not investigated, is is from this shortage of data possible to make following assumption:

$$
\begin{equation}
\lim_{precision \rightarrow \infty} \lVert \varepsilon \rVert \rightarrow 0.
\end{equation}
$$

The number of zeros column and the NaN column, those numbers are excluded from the averaging. The averaging is of 100000 points, so that the total number of samples is then 100000 plus the zero and NaN count.

The high error in the single precision case have not been investigated, and is to be considered a todo item.

## Discussion

The literature survey was limited since it would be too much effort to read up on a complete new field and the science related to the Möbius transformation. So there might have been algorithms which cover the same problem but was not found. The literature tells that all transformation inside the pointcaré disk model are Möbius transformations, so it should be possible to express the theorem using Möbius transformations.

With higher precision the results approaches zero. That makes the argument that the geometric construction is exact, even if the author have not been able to prove it theoretical. The most important part is to considering the decrease of magnitude of the error since they may vary between executions.

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