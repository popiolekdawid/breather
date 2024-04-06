#include <gtk/gtk.h>
#include <cairo.h>
#include <math.h>
#include "graph.h"

static gboolean draw_callback(GtkWidget *widget, cairo_t *cr, gpointer data) {
    gint width, height;
    gtk_widget_get_size_request(widget, &width, &height);
    Graph *graph = (Graph *)data;

    cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
    cairo_paint(cr);

    cairo_set_line_width(cr, 1);
    cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
    
    cairo_move_to(cr, 0, height / 2);
    cairo_line_to(cr, width, height / 2);   

    cairo_stroke(cr);
    
    cairo_set_line_width(cr, 2);
    cairo_set_source_rgb(cr, 1.0, 0.0, 0.0); 
    
    cairo_move_to(cr, 0, height - (graph->y_values[0] * height / graph->max_y_deviation));
    for (int x = 0; x < graph->y_count; x++) {
        cairo_line_to(cr, x * width / graph->y_count, height - (graph->y_values[x] * height / graph->max_y_deviation));
    }
    cairo_stroke(cr);

    return FALSE;
}

static gboolean update_graph(gpointer data) {
    Graph *graph = (Graph *)data;

    for(int i = 0; i < graph->y_count - 1; i++){
        graph->y_values[i] = graph->y_values[i+1];
    }
    graph->y_values[graph->y_count - 1] = *graph->source;

    gtk_widget_queue_draw(GTK_WIDGET(graph->drawing_area));
    return G_SOURCE_CONTINUE;
}


Graph *graph_new(int width, int height, int y_count, int max_y_deviation, double *source) {
    Graph *graph = (Graph *)malloc(sizeof(Graph));
    graph->y_count = y_count;
    graph->y_values = (int *)calloc(graph->y_count,sizeof(int));
    graph->max_y_deviation = max_y_deviation;
    graph->source = source;

    for(int i = 0; i < graph->y_count; i++){
        graph->y_values[i] = 0;
    }

    graph->drawing_area = gtk_drawing_area_new();
    gtk_widget_set_size_request(graph->drawing_area, width, height);
    g_signal_connect(G_OBJECT(graph->drawing_area), "draw", G_CALLBACK(draw_callback), graph);
    g_timeout_add(30, update_graph, graph);

    return graph;
}
