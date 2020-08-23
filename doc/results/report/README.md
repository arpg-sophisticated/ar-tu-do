# Final report data

This directory contains the data used for the final report:
+ laptimes-gazebo-wf2.txt - Lap times of ten laps running Wallfollowing 2 in Gazebo
+ laptimes-gazebo-wf5.txt - Lap times of ten laps running Wallfollowing 5 in Gazebo
+ laptimes-gazebo-ql.txt - Lap times of ten laps running QL in Gazebo
* latex-data-QL-gazebo-1000.dat - This test was done after some changes on speed calculation, please loot at the latex template for simulation QL to get the right correction factors
* latex-data-WF2-1000.dat - This data was recorded during a simulation based test with Wallfollowing 2
* latex-data-WF5-1000.dat - This data was recorded during a simulation based test with Wallfollowing 5
* latex-data-WF2-500-real.dat - This data was recorded during a real racing test with Wallfollowing 2
* latex-data-WF5-500-real.dat - This data was recorded during a real racing test with Wallfollowing 5

## Latex/PGFPlot Code

### Simulation WF2/WF5

```
\begin{figure}
\label{fig:results-sim-wf-plot}
\center
    \begin{tikzpicture}
        \begin{axis}[width=1\textwidth, height=0.3\paperheight, scaled ticks=true, tick label style={/pgf/number format/fixed, font=\footnotesize}, 
        %axis lines=middle, 
        %ymode=log,
        ymin=0, 
        xmin=0, 
        xmax=250,
        xlabel=\footnotesize $m$, ylabel=\footnotesize $\frac{m}{s}$,
        xtick distance=25,
        ]
        
            \addplot[smooth,red,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=speed,x=distance]{data/latex-data-WF2-1000.dat};
            \addplot[smooth,blue,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=speed,x=distance]{data/latex-data-WF5-1000.dat};
            \addlegendentry{\footnotesize WF2 $v_{cur}(d)$}
            \addlegendentry{\footnotesize WF5 $v_{cur}(d)$}
        \end{axis}
    \end{tikzpicture}
    \caption{Vergleich Geschwindigkeit $v$ nach gefahrener Distanz $d$, geglättet über 5 Werte (Simulation, Softwarestand v0.42g1)}
\end{figure}
```

### Simulation QL
```
\begin{figure}
\label{fig:results_ql}
\center
    \begin{tikzpicture}
        \begin{axis}[width=1\textwidth, height=0.3\paperheight, scaled ticks=true, tick label style={/pgf/number format/fixed, font=\footnotesize}, 
        %axis lines=middle, 
        %ymode=log,
        ymin=0, 
        xmin=0,
        xmax=250,
        xlabel=\footnotesize $m$, ylabel=\footnotesize $\frac{m}{s}$,
        xtick distance=25,
        ]
            \addplot[smooth,red,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=speed,x=distance]{data/latex-data-WF2-1000.dat};
            \addplot[smooth,blue,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=speed,x=distance]{data/latex-data-WF5-1000.dat};
            \addplot[smooth,green,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y expr=\thisrow{speed}*1.1111,x expr=(\thisrow{distance}*1.1111)+2.778]{data/latex-data-QL-gazebo-1000.dat};
            \addlegendentry{\footnotesize WF2 $v_{cur}(d)$}
            \addlegendentry{\footnotesize WF5 $v_{cur}(d)$}
            \addlegendentry{\footnotesize QL $v_{cur}(d)$}
        \end{axis}
    \end{tikzpicture}
    \caption{Vergleich Geschwindigkeit $v$ nach gefahrener Distanz $d$, geglättet über 5 Werte (Simulation, Softwarestand v0.42q1)}
\end{figure}
```

### Racing Test WF2/WF5
```
\begin{figure}
\label{fig:results_wf2_plot}
\center
    \begin{tikzpicture}
        \begin{axis}[width=1\textwidth, height=0.3\paperheight, scaled ticks=true, tick label style={/pgf/number format/fixed, font=\footnotesize}, 
        %axis lines=middle, 
        %ymode=log,
        %ymin=0, 
        xmin=0, 
        xmax=75,
        xlabel=\footnotesize $m$, ylabel=\footnotesize $\frac{m}{s}$,
        xtick distance=10,
        ]
        
            \addplot[smooth,red,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y expr=\thisrow{speed}*0.9,x=distance]{data/latex-data-WF2-500-real.dat};
            \addlegendentry{\footnotesize WF2 $v_{cur}(d)$}
        \end{axis}
    \end{tikzpicture}
    \caption{WF2: Geschwindigkeit $v$ nach gefahrener Distanz $d$, geglättet über 5 Werte (Softwarestand v0.42r1)}
\end{figure}

\begin{figure}
\label{fig:results_wf5_plot}
\center
    \begin{tikzpicture}
        \begin{axis}[width=1\textwidth, height=0.3\paperheight, scaled ticks=true, tick label style={/pgf/number format/fixed, font=\footnotesize}, 
        %axis lines=middle, 
        %ymode=log,
        %ymin=0, 
        xmin=0, 
        xmax=90,
        xlabel=\footnotesize $m$, ylabel=\footnotesize $\frac{m}{s}$,
        xtick distance=10,
        ]
        
            \addplot[smooth,red,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=speed,x=distance]{data/latex-data-WF5-500-real.dat};
            \addlegendentry{\footnotesize WF5 $v_{cur}(d)$}
        \end{axis}
    \end{tikzpicture}
    \caption{WF5: Geschwindigkeit $v$ nach gefahrener Distanz $d$, geglättet über 5 Werte (Softwarestand v0.42r2)}
\end{figure}

\begin{figure}
\label{fig:results_wf2_plot_vmax}
\center
    \begin{tikzpicture}
        \begin{axis}[width=1\textwidth, height=0.3\paperheight, scaled ticks=true, tick label style={/pgf/number format/fixed, font=\footnotesize}, 
        %axis lines=middle, 
        ymode=log,
        %ymin=0, 
        xmin=0, 
        xmax=75,
        xlabel=\footnotesize $m$, ylabel=\footnotesize $\frac{m}{s}$,
        xtick distance=10,
        ]
        
            \addplot[smooth,red,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y expr=\thisrow{speed}*0.9,x=distance]{data/latex-data-WF2-500-real.dat};
            \addplot[smooth,blue,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=maxspeed,x=distance]{data/latex-data-WF2-500-real.dat};
            \addlegendentry{\footnotesize WF2 $v_{cur}(d)$}
            \addlegendentry{\footnotesize WF2 $v_{max}(d)$}
        \end{axis}
    \end{tikzpicture}
    \caption{WF2: Geschwindigkeiten $v_{cur} und v_{max}$ nach gefahrener Distanz $d$, geglättet über 5 Werte (Softwarestand v0.42r1)}
\end{figure}

\begin{figure}
\label{fig:results_wf5_plot_vmax}
\center
    \begin{tikzpicture}
        \begin{axis}[width=1\textwidth, height=0.3\paperheight, scaled ticks=true, tick label style={/pgf/number format/fixed, font=\footnotesize}, 
        %axis lines=middle, 
        ymode=log,
        %ymin=0, 
        xmin=0, 
        xmax=90,
        xlabel=\footnotesize $m$, ylabel=\footnotesize $\frac{m}{s}$,
        xtick distance=10,
        ]
        
            \addplot[smooth,red,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=speed,x=distance]{data/latex-data-WF5-500-real.dat};
            \addplot[smooth,blue,solid,each nth point=5, filter discard warning=false, unbounded coords=discard] table [y=maxspeed,x=distance]{data/latex-data-WF5-500-real.dat};
            \addlegendentry{\footnotesize WF5 $v_{cur}(d)$}
            \addlegendentry{\footnotesize WF5 $v_{max}(d)$}
        \end{axis}
    \end{tikzpicture}
    \caption{WF5: Geschwindigkeiten $v_{cur} und v_{max}$ nach gefahrener Distanz $d$, geglättet über 5 Werte (Softwarestand v0.42r2)}
\end{figure}
```
