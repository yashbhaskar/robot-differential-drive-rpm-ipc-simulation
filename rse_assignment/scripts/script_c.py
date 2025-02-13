#!/usr/bin/env python3

import requests
import time
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import threading
import numpy as np

left_rpm_values = []
right_rpm_values = []
timestamps = []

# Function to fetch data from Script B's API
def fetch_data_from_script_b():
    try:
        response = requests.get("http://localhost:8080/get_wheel_data")  # Update the URL if needed
        if response.status_code == 200:
            return response.json()  # Return the fetched JSON data
        else:
            print("Error fetching data:", response.status_code)
            return None
    except Exception as e:
        print(f"Exception occurred: {e}")
        return None

# Function to continuously fetch data and store it
def data_fetch_loop():
    while True:
        data = fetch_data_from_script_b()
        if data:
            left_rpm = data.get("left_rpm", 0)
            right_rpm = data.get("right_rpm", 0)
            timestamp_script_a = data.get("timestamp_script_a", 0)

            left_rpm_values.append(left_rpm)
            right_rpm_values.append(right_rpm)
            timestamps.append(timestamp_script_a)

            print(f"Left RPM: {left_rpm}, Right RPM: {right_rpm}, Timestamp: {timestamp_script_a}")

            if len(left_rpm_values) > 60:
                left_rpm_values.pop(0)
                right_rpm_values.pop(0)
                timestamps.pop(0)

        time.sleep(1)  # Fetch data every 1 second

# Start the data fetch loop in a separate thread
fetch_thread = threading.Thread(target=data_fetch_loop, daemon=True)
fetch_thread.start()

# Initialize Dash app
app = dash.Dash(__name__)

# Layout of the dashboard
app.layout = html.Div(children=[
    html.H1("Real-Time Wheel RPM Data", style={'textAlign': 'center', 'color': 'white'}),
    
    # Graph for live data
    dcc.Graph(id='live-graph'),
    
    dcc.Interval(id='interval-component', interval=1000, n_intervals=0),  

    html.Div(children=[
        html.Div(children=[
            html.H4("Latest RPM Data", style={'color': 'white'}),
            html.P(id='latest-rpm-data', style={'color': 'white'}),
        ], style={'display': 'inline-block', 'width': '45%', 'padding': '10px'}),
        
        html.Div(children=[
            html.H4("Statistics", style={'color': 'white'}),
            html.P(id='rpm-stats', style={'color': 'white'}),
        ], style={'display': 'inline-block', 'width': '45%', 'padding': '10px'}),
    ], style={'display': 'flex', 'justifyContent': 'space-between', 'marginTop': '30px', 'backgroundColor': '#2D2D2D', 'padding': '20px', 'borderRadius': '10px'}),

    # Footer with system information or credits
    html.Div(children=[
        html.Div(children=[
            html.P("System Status: ", style={'color': 'white', 'marginBottom': '0px'}),
            html.P("Running", style={'color': 'green', 'marginTop': '0px'}),
        ], style={'display': 'inline-block', 'width': '30%', 'padding': '10px'}),
        
        html.Div(children=[
            html.P("Version: ", style={'color': 'white', 'marginBottom': '0px'}),
            html.P("1.0.0", style={'color': 'white', 'marginTop': '0px'}),
        ], style={'display': 'inline-block', 'width': '30%', 'padding': '10px'}),
        
        html.Div(children=[
            html.P("Contact: ", style={'color': 'white', 'marginBottom': '0px'}),
            html.P("ybbhaskar19@gmail.com", style={'color': 'white', 'marginTop': '0px'}),
        ], style={'display': 'inline-block', 'width': '30%', 'padding': '10px'}),
    ], style={'display': 'flex', 'justifyContent': 'space-between', 'marginTop': '30px', 'backgroundColor': '#2D2D2D', 'padding': '20px', 'borderRadius': '10px'}),

    # Footer with additional information or credits
    html.Div(children=[
        html.P("Dashboard powered by Yash", style={'textAlign': 'center', 'color': 'white', 'marginTop': '40px', 'fontSize': '14px'}),
    ], style={'backgroundColor': '#2D2D2D', 'padding': '10px'})
])

# Callback to update graph
@app.callback(
    Output('live-graph', 'figure'),
    Input('interval-component', 'n_intervals')
)
def update_graph(n):
    if len(timestamps) == 0:
        return {
            'data': [],
            'layout': go.Layout(
                title='Wheel RPMs Over Time',
                xaxis_title="Time (ms)",
                yaxis_title="RPM",
                template="plotly_dark",
                xaxis=dict(showline=True, linewidth=2, linecolor='white'),
                yaxis=dict(showline=True, linewidth=2, linecolor='white')
            )
        }

    return {
        'data': [
            go.Scatter(x=timestamps, y=left_rpm_values, mode='lines+markers', name='Left RPM', line=dict(color='blue')),
            go.Scatter(x=timestamps, y=right_rpm_values, mode='lines+markers', name='Right RPM', line=dict(color='red'))
        ],
        'layout': go.Layout(
            title='Wheel RPMs Over Time',
            xaxis_title="Time (ms)",
            yaxis_title="RPM",
            template="plotly_dark",
            xaxis=dict(showline=True, linewidth=2, linecolor='white'),
            yaxis=dict(showline=True, linewidth=2, linecolor='white')
        )
    }

# Callback to update the latest RPM data and statistics
@app.callback(
    [Output('latest-rpm-data', 'children'),
     Output('rpm-stats', 'children')],
    Input('interval-component', 'n_intervals')
)
def update_stats(n):
    if len(left_rpm_values) == 0 or len(right_rpm_values) == 0:
        latest_rpm_data = "No data yet."
        rpm_stats = "No data yet."
    else:
        latest_left_rpm = left_rpm_values[-1]
        latest_right_rpm = right_rpm_values[-1]
        
        avg_left_rpm = np.mean(left_rpm_values) if left_rpm_values else 0
        avg_right_rpm = np.mean(right_rpm_values) if right_rpm_values else 0

        latest_rpm_data = f"Left RPM: {latest_left_rpm}, Right RPM: {latest_right_rpm}"
        rpm_stats = f"Average Left RPM: {avg_left_rpm:.2f}, Average Right RPM: {avg_right_rpm:.2f}"

    return latest_rpm_data, rpm_stats

# Run the app
if __name__ == '__main__':
    app.run_server(debug=True, host="0.0.0.0", port=8050)
