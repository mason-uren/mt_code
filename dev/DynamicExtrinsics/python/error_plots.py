import numpy as np
from measure_model_error import *
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import seaborn as sns
import pandas as pd

def error_vs_pan_tilt_heat(rel_ext_model, datasets, labels, prefix, err_fn=None, 
                           plot_corner_dist=False, interactive_3d=False):
    if err_fn is None:
        err_fn = static_exploded_norms
    
    file_ext = ".pdf"
    
    scale=1
    figsize = (scale*6.4, scale*4.8)
    plt.figure('heat', figsize=figsize)
    plt.figure('dist', figsize=figsize)
    plt.figure('corners', figsize=figsize)
    plt.figure('cdf', figsize=figsize)
        
    dfs = []
    for dataset, label in zip(datasets, labels):
        err_norms = err_fn(rel_ext_model, dataset) * 1000 #convert to mm from meters
        #pdb.set_trace()
        dfs.append(pd.DataFrame(data=dict(
            err=err_norms,
            pan=dataset['pan_tilt'][:,0],
            tilt=dataset['pan_tilt'][:,1],
            label=label
        )))
        
        plt.figure('dist')
        sns.distplot(err_norms,
                     label=label)
        
        plt.figure('cdf')
        
    df = pd.concat(dfs, ignore_index=True)
    
    dfs_corners = []
    if plot_corner_dist:
        for dataset, label in zip(datasets, labels):
            if 'num_valids_ximea' in dataset:
                num_corners = dataset['num_valids_ximea']
                #print('num_valids_ximea', num_corners)
                dfs_corners.append(pd.DataFrame(data=dict(
                    corners=num_corners,
                    pan=dataset['pan_tilt'][:,0],
                    tilt=dataset['pan_tilt'][:,1],
                    label=label
                )))
        
    """
    Histogram (pdf)
    """
    
    plt.figure('dist')
    plt.xlabel('Error (mm)')
    plt.ylabel('Density')
    plt.xlim((0,min(10, df.err.max())))
    plt.legend()
    plt.savefig('{}-dist{}'.format(prefix, file_ext))
    
    plt.clf()
    
    """
    Histogram (cumulative)
    """
    
    plt.figure('cdf')
    sns.distplot(df.err,
                 bins=int(100 * (df.err.max() - df.err.min())),
                 hist_kws={'cumulative': True},
                 kde_kws={'cumulative': True, 'bw': 0.01},
                 label=label)
    # plt.gca().set_xscale('log')
    # plt.gca().xaxis.set_minor_formatter(mticker.ScalarFormatter())
    plt.xlim((0,5))
    plt.xlabel('Error (mm)')
    plt.ylabel('Cumulative Probability')
    plt.grid()
    plt.savefig('{}-cdf{}'.format(prefix, file_ext))
    
    plt.clf()
    
    """
    Heatmap scatterplot thing
    """
    # replace error outliers with the max error after taking out the outliers
    # so as to not to affect the scale of the plot by the outliers
    df["outlier"] = df.err > 10
    #df.loc[df.outlier, 'err'] = 10.0
    df.loc[df.outlier, 'err'] = 0.0
    max_err = df.err.max()
    df.loc[df.outlier, 'err'] = max_err

    plt.figure('heat')
    sns.scatterplot(data=df, x='pan', y='tilt', hue='label', size=df.err**2,
                    sizes=(1, 100), style='outlier', legend="brief")
    plt.xlabel('Pan')
    plt.ylabel('Tilt')
    legends = plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    for text in legends.get_texts():
        content = text.get_text()
        try:
            text.set_text(round(float(content)**0.5, 1))
        except ValueError:
            pass
        
    ax = plt.gca()
    ax.set_xlim(ax.get_xlim()[::-1])
    ax.set_ylim(ax.get_ylim()[::-1])
    
    plt.savefig('{}-heat{}'.format(prefix, file_ext), bbox_inches='tight')
    plt.clf()

    """
    heatmap for the number of detected chessboard corners as a proxy for FOV coverage
    """
    try:
        dfs_corners = pd.concat(dfs_corners, ignore_index=True)
        plt.figure('corners')
        sns.scatterplot(data=dfs_corners, x='pan', y='tilt', hue='label', size=dfs_corners.corners/5,
                        sizes=(20, 100), legend="brief")
        plt.xlabel('Pan')
        plt.ylabel('Tilt')
        legends = plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        for text in legends.get_texts():
            content = text.get_text()
            try:
                text.set_text(round(float(content)*5))
            except ValueError:
                pass
        ax = plt.gca()
        ax.set_xlim(ax.get_xlim()[::-1])
        ax.set_ylim(ax.get_ylim()[::-1])
        plt.savefig('{}-ximea-corners{}'.format(prefix, file_ext), bbox_inches='tight')
        plt.clf()
    except: # no data in dfs_fov
        pass

    ''' plotting error heatmap in 3D surface plots
    '''
    if interactive_3d:
        plt.figure('heat')
        plt.close()
        plt.figure('cdf')
        plt.close()
        plt.figure('corners')
        plt.close()
        plt.figure('dist')
        plt.close()
        fig = plt.figure('Error Surface Plot (Interactive)')
        ax = plt.axes(projection='3d')
        ax.plot_trisurf(df['pan'],df['tilt'],df['err'],
                        cmap='viridis', edgecolor='none');
        plt.show()
