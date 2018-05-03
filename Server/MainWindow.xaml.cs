using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace ServerApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private Server server;
        private Visualization visualization;
        private bool connected = false;
        private ObservableCollection<string> connectedDevices = new ObservableCollection<string>();

        public MainWindow()
        {
            InitializeComponent();
            server = new Server();
            server.ShowNotificationEventHandler += OnShowNotification;
            server.ChangeStatusEventHandler += OnChangeStatus;
            server.UpdateConnectedDevicesEventHandler += OnUpdateConnectedDevices;
            server.UpdateTrackedNumberHandler += OnUpdateTrackedNumber;
            visualization = new Visualization(this.winFormsHost, server);
            ipAddressesComboBox.ItemsSource = server.getPossibleIpAddresses();
            ipAddressesComboBox.SelectedIndex = 0;
            portTxtBox.Text = "" + Server.SERVER_PORT;
            connectedDevicesLV.ItemsSource = connectedDevices;
        }

        public void OnShowNotification(object sender, EventArgs e)
        {
            Dispatcher.Invoke(() =>
            {
                Storyboard sb = this.notificationEllipse.FindResource("notificationStoryBoard") as Storyboard;
                sb.Begin();
            });
        }

        public void OnChangeStatus(object sender, ChangeStatusEventArgs e)
        {
            Dispatcher.Invoke(() => { statusBar.Text = e.Status; });
        }

        public void OnUpdateConnectedDevices(object sender, UpdateConnectedDevicesEventArgs e)
        {
            Dispatcher.Invoke(() =>
            {
                if (e.Add != null && !this.connectedDevices.Contains(e.Add))
                {
                    this.connectedDevices.Add(e.Add);
                }
                if (e.Remove != null && this.connectedDevices.Contains(e.Remove))
                {
                    this.connectedDevices.Remove(e.Remove);
                }
            });
        }

        public void OnUpdateTrackedNumber(object sender, UpdateTrackedNumberEventArgs e)
        {
            Dispatcher.Invoke(() => { this.trackedPeopleLbl.Content = e.TrackedNumber; });
        }

        private void StartButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (!connected)
                {
                    int port = int.Parse(portTxtBox.Text);
                    server.StartListening((IPAddress)(ipAddressesComboBox.SelectedItem), port);
                    startBtn.Content = "Disconnect";
                    connected = true;
                }
                else
                {
                    server.Close();
                    startBtn.Content = "Start listening";
                    connected = false;
                }
            }
            catch (Exception exception)
            {
                MessageBox.Show(exception.ToString());
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (server != null)
            {
                server.Close();
            }
        }
    }
}