#include <gtk/gtk.h>
#include "graph.h"
#define HEIGHT 1000
#define WIDTH 1600

#define MIN_PRESSURE 0
#define MAX_PRESSURE 200
#define MIN_VOLUME 1
#define MAX_VOLUME 500
#define MIN_PEEP 0
#define MAX_PEEP 40
#define MIN_INHALE_EXHALE_RATIO 0
#define MAX_INHALE_EXHALE_RATIO 4
#define MIN_BREATHS_PER_MINUTE 0
#define MAX_BREATHS_PER_MINUTE 200


//VARIABLES AND DATATYPES
typedef struct{
	uint8_t mode;
	double pressure;
	double volume;
	double flow;
	double peep;
	double inhale_exhale_ratio;
	double breaths_per_minute;
	double pv;
	int inhale_time;
	int exhale_time;
} Data;

typedef struct{
	double pressure;
	double volume;
	double peep;
	double inhale_exhale_ratio;
	double breaths_per_minute;
} InputData;

typedef struct{
	//Layouts
	GtkWidget *window;
	GtkWidget *main_layout;
	GtkWidget *graph_layout;
	GtkWidget *setting_layout;
	
	//Widgets
	//~~~~Buttons
	GtkWidget *update_data_button;
	GtkWidget *exit_button;
	GtkWidget *pressure_volume_button;
	//~~~~Scales
	GtkWidget *pressure_volume_scale;
	GtkWidget *peep_scale;
	GtkWidget *inhale_exhale_scale;
	GtkWidget *breaths_per_minute_scale;
	//~~~~Graphs
	Graph *pressure_graph;
	Graph *flow_graph;
	Graph *volume_graph;
	Graph *pv_graph;
	//~~~~Labels
	GtkWidget *pressure_volume_label;
	GtkWidget *peep_label;
	GtkWidget *inhale_exhale_label;
	GtkWidget *breaths_per_minute_label;

	//Variables	
	Data *data;
	InputData *input_data;
}UserInterface;


//FUNCTIONS
static void activate (GtkApplication* app);
static gboolean send_work_settings (gpointer data);
static void update_data (GtkButton *button, gpointer *data);
static void pressure_volume_change (GtkButton *button, gpointer *data);
static void pressure_volume_scale_changed (GtkRange *range, gpointer *data);
static void peep_scale_changed (GtkRange *range, gpointer *data);
static void inhale_exhale_scale_changed (GtkRange *range, gpointer *data);
static void breaths_per_minute_scale_changed (GtkRange *range, gpointer *data);


//MAIN
int main (int argc, char **argv){
	GtkApplication *app;
	int app_status;

	app = gtk_application_new ("com.student.controlapp", G_APPLICATION_DEFAULT_FLAGS);
	g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
	app_status = g_application_run (G_APPLICATION (app),argc,argv);
	g_object_unref (app);

	return app_status;
}


static void activate (GtkApplication* app){
	
	//--------------------DEFINITION
	UserInterface *ui;


	//--------------------INITIALIZATIONS
	ui = (UserInterface *)malloc(sizeof(UserInterface));
	
	//Layouts
	ui->window = gtk_application_window_new(app);
	ui->main_layout = gtk_grid_new();
	ui->graph_layout = gtk_grid_new();
	ui->setting_layout = gtk_box_new(GTK_ORIENTATION_VERTICAL,5);

	//Variables
	ui->data = (Data *)malloc(sizeof(Data));
	ui->data->mode = 1;
	ui->data->pressure = 0;
	ui->data->volume = 1;
	ui->data->flow = 0;
	ui->data->peep = 0;
	ui->data->inhale_exhale_ratio = 0;
	ui->data->breaths_per_minute = 0;
	ui->data->inhale_time = 0;
	ui->data->exhale_time = 0;
	ui->data->pv = 1;

	ui->input_data = (InputData *)malloc(sizeof(InputData));
	ui->input_data->pressure = 0;
	ui->input_data->volume = 1;
	ui->input_data->peep = 0;
	ui->input_data->inhale_exhale_ratio = 0;
	ui->input_data->breaths_per_minute = 0;

	//Widgets
	//~~~~Buttons
	ui->update_data_button = gtk_button_new_with_label("Update Data");
	ui->exit_button = gtk_button_new_with_label("Exit");
	ui->pressure_volume_button = gtk_button_new_with_label("Pressure Mode");
	//~~~~Scales
	ui->pressure_volume_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, MIN_PRESSURE, MAX_PRESSURE, 1);
	ui->peep_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, MIN_PEEP, MAX_PEEP, 1);
	ui->inhale_exhale_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, MIN_INHALE_EXHALE_RATIO, MAX_INHALE_EXHALE_RATIO, 0.1);
	ui->breaths_per_minute_scale = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, MIN_BREATHS_PER_MINUTE, MAX_BREATHS_PER_MINUTE, 1);
	//~~~~Graphs
	ui->pressure_graph = graph_new(400,400,100,MAX_PRESSURE,&ui->data->pressure);
	ui->flow_graph = graph_new(400,400,100,MAX_VOLUME,&ui->data->flow);
	ui->volume_graph = graph_new(400,400,100,MAX_VOLUME,&ui->data->volume);
	ui->pv_graph = graph_new(400,400,100,MAX_PRESSURE,&ui->data->pv);
	//~~~~Labels
	ui->pressure_volume_label = gtk_label_new("Pressure:");
	ui->peep_label = gtk_label_new("PEEP:");
	ui->inhale_exhale_label = gtk_label_new("Inhale/Exhale Ratio:");
	ui->breaths_per_minute_label = gtk_label_new("Breaths per Minute:");
	
	//Serial with STM32F411



	//--------------------LAYOUTS ADDING AND COMPOSITION
	
	//Window and Main Layout
	gtk_container_add(GTK_CONTAINER(ui->window),ui->main_layout);
	gtk_grid_attach(GTK_GRID(ui->main_layout),ui->update_data_button,0,0,1,1);
	gtk_grid_attach(GTK_GRID(ui->main_layout),ui->exit_button,1,0,1,1);
	gtk_grid_attach(GTK_GRID(ui->main_layout),ui->setting_layout,0,1,1,1);
	gtk_grid_attach(GTK_GRID(ui->main_layout),ui->graph_layout,1,1,1,1);

	//Settings Layout
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->pressure_volume_button,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->pressure_volume_label,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->pressure_volume_scale,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->inhale_exhale_label,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->inhale_exhale_scale,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->peep_label,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->peep_scale,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->breaths_per_minute_label,1,1,0);
	gtk_box_pack_start(GTK_BOX(ui->setting_layout),ui->breaths_per_minute_scale,1,1,0);
	
	//Graph Layout
	gtk_grid_attach(GTK_GRID(ui->graph_layout),ui->pressure_graph->drawing_area,0,0,1,1);
	gtk_grid_attach(GTK_GRID(ui->graph_layout),ui->flow_graph->drawing_area,1,0,1,1);
	gtk_grid_attach(GTK_GRID(ui->graph_layout),ui->volume_graph->drawing_area,0,1,1,1);
	gtk_grid_attach(GTK_GRID(ui->graph_layout),ui->pv_graph->drawing_area,1,1,1,1);



	//--------------------FUNCTIONALITIES
	
	//Scales
	g_signal_connect (GTK_RANGE(ui->pressure_volume_scale), "value_changed", G_CALLBACK (pressure_volume_scale_changed), ui);
	g_signal_connect (GTK_RANGE(ui->peep_scale), "value_changed", G_CALLBACK (peep_scale_changed), ui);
	g_signal_connect (GTK_RANGE(ui->inhale_exhale_scale), "value_changed", G_CALLBACK (inhale_exhale_scale_changed), ui);
	g_signal_connect (GTK_RANGE(ui->breaths_per_minute_scale), "value_changed", G_CALLBACK (breaths_per_minute_scale_changed), ui);

	//Buttons
	g_signal_connect (GTK_BUTTON(ui->update_data_button), "clicked", G_CALLBACK (update_data), ui);
	g_signal_connect (GTK_BUTTON(ui->exit_button), "clicked", G_CALLBACK (gtk_window_close), ui->window);
	g_signal_connect (GTK_BUTTON(ui->pressure_volume_button), "clicked", G_CALLBACK (pressure_volume_change), ui);

	//Misc
	g_timeout_add(10, G_SOURCE_FUNC(send_work_settings), ui);


	//--------------------CUSTOMIZATION OPTIONS
	
	//Layouts
	gtk_grid_set_column_homogeneous(GTK_GRID(ui->main_layout),1);
	gtk_box_set_homogeneous(GTK_BOX(ui->setting_layout),1);
	gtk_grid_set_row_homogeneous(GTK_GRID(ui->graph_layout),1);
	gtk_grid_set_column_homogeneous(GTK_GRID(ui->graph_layout),1);

	//Window
	gtk_window_set_title(GTK_WINDOW(ui->window), "Control Aplication");
	gtk_window_maximize(GTK_WINDOW(ui->window));
	gtk_window_set_default_size(GTK_WINDOW(ui->window), HEIGHT, WIDTH);
	gtk_widget_show_all(ui->window);
}



//FUNCTION IMPLEMENTATIONS
static gboolean send_work_settings (gpointer data){
	UserInterface *ui = (UserInterface *)data;

}

static void update_data (GtkButton *button, gpointer *data){
	UserInterface *ui = (UserInterface *)data;
	

	//Logic for sending new data
	ui->data->pressure = ui->input_data->pressure;
	ui->data->volume = ui->input_data->volume;
	ui->data->peep = ui->input_data->peep;
	ui->data->breaths_per_minute = ui->input_data->breaths_per_minute;
	ui->data->inhale_exhale_ratio = ui->input_data->inhale_exhale_ratio;
	ui->data->pv = ui->data->pressure / ui->data->volume;
}

static void pressure_volume_change (GtkButton *button, gpointer *data){
	UserInterface *ui = (UserInterface *)data;
	ui->data->mode ^= 1;
	if(ui->data->mode){
		gtk_label_set_text(GTK_LABEL(ui->pressure_volume_label), "Pressure:");
		gtk_range_set_value(GTK_RANGE(ui->pressure_volume_scale), ui->input_data->pressure);
		gtk_range_set_range(GTK_RANGE(ui->pressure_volume_scale), MIN_PRESSURE, MAX_PRESSURE);
		gtk_button_set_label(GTK_BUTTON(button), "Pressure Mode");
	}else{
		gtk_label_set_text(GTK_LABEL(ui->pressure_volume_label), "Volume:");
		gtk_range_set_range(GTK_RANGE(ui->pressure_volume_scale), MIN_VOLUME, MAX_VOLUME);
		gtk_range_set_value(GTK_RANGE(ui->pressure_volume_scale), ui->input_data->volume);
		gtk_button_set_label(GTK_BUTTON(button), "Volume Mode");
	}
}

static void pressure_volume_scale_changed (GtkRange *range, gpointer *data){
	UserInterface *ui = (UserInterface *)data;
	if(ui->data->mode){ //pressure mode
		ui->input_data->pressure = gtk_range_get_value(range);
	}else{ //volume mode
		ui->input_data->volume = gtk_range_get_value(range);
	}
}

static void peep_scale_changed (GtkRange *range, gpointer *data){
	UserInterface *ui = (UserInterface *)data;
	ui->input_data->peep = gtk_range_get_value(range);
}

static void inhale_exhale_scale_changed (GtkRange *range, gpointer *data){
	UserInterface *ui = (UserInterface *)data;
	ui->input_data->inhale_exhale_ratio = gtk_range_get_value(range);
}

static void breaths_per_minute_scale_changed (GtkRange *range, gpointer *data){
	UserInterface *ui = (UserInterface *)data;
	ui->input_data->breaths_per_minute = gtk_range_get_value(range);
}