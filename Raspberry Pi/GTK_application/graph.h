#ifndef GRAPH_H
#define GRAPH_H
#include <gtk/gtk.h>

typedef struct {
    int y_count;
    int *y_values;
    int max_y_deviation;
    GtkWidget *drawing_area;
    double *source;
} Graph;


Graph *graph_new(int width, int height, int y_count, int max_y_deviation, double *source);


static gboolean draw_callback(GtkWidget *widget, cairo_t *cr, gpointer data);
static gboolean update_graph(gpointer data);

#endif