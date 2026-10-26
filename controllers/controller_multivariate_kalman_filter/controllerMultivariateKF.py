from controller import Robot
from controller import Supervisor
import csv
import numpy as np
import matplotlib.pyplot as plt


# Kalman Filter
# It predicts the robot's next position using the motion model and corrects it using GPS measurements.
# This helps us estimate the robot's position more reliably than using GPS readings alone.
class KalmanFilter() :
    def __init__(self, stdX, stdY, stdXVel, stdYVel, stdZX, stdZY, dt = 0.064, xDim = 4, zDim = 2, stdAcc = 0.5, F = None, H = None, U = 0.) :
        self.xDim = xDim
        self.zDim = zDim

        # Robot's initial Position
        self.xInitial = np.array([[0.], [0.], [0.], [0.]])

        self.x = self.xInitial
        self.dt = dt

        self.xPrior = self.xInitial.copy()
        self.xPosterior = self.xInitial.copy()

        self.PPrior = np.zeros((4, 4))
        self.PPosterior = np.zeros((4, 4))

        # State Transition function
        if F is None :
            self.F = np.array([[1., dt, 0., 0.], [0., 1., 0., 0.], [0., 0., 1., dt], [0., 0., 0., 1]])
        else :
            self.F = F

        # State Uncertainty
        self.P = np.array([[stdX**2, 0., 0., 0.], [0., stdXVel**2, 0., 0.], [0., 0., stdY**2, 0.], [0., 0., 0., stdYVel**2]])

        # Standard deviation of the unmodelled acceleration (m/s^2)
        stdAcc = stdAcc

        dt = self.dt

        q = stdAcc**2

        self.Q = q * np.array([
            [dt**4/4, dt**3/2, 0.,       0.],
            [dt**3/2, dt**2,   0.,       0.],
            [0.,      0.,      dt**4/4,  dt**3/2],
            [0.,      0.,      dt**3/2,  dt**2]
            ])

        # Measurement function
        self.H = np.array([[1., 0., 0., 0.], [0., 0., 1., 0.]])

        # Measurement Uncertainty
        self.R = np.array([[stdZX**2, 0.], [0., stdZY**2]])

        # Residual
        self.y = np.zeros((2, 1))

        # Innovation Covariance
        self.S = np.zeros((2, 2))

        # Kalman Gain
        self.K = np.zeros((4, 2))

    def predict(self) :
        # Predict the next state and its uncertainty using the motion model.
        # We use this prediction before correcting it with the GPS measurement.
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        self.xPrior = self.x.copy()
        self.PPrior = self.P.copy()

        return self.xPrior, self.PPrior

    def update(self, z) :

        # Calculate the difference between the GPS measurement and the predicted position.
        y = z - np.dot(self.H, self.x)
        self.y = y.copy()

        # Kalman Gain
        # It determines how much we should trust the prediction and the GPS measurement.
        PdotHT = np.dot(self.P, self.H.T)

        S = np.dot(self.H, PdotHT) + self.R
        self.S = S.copy()

        K = np.dot(PdotHT, np.linalg.inv(S))
        self.K = K.copy()

        # Update Step
        # Correct the predicted state using the residual and update its uncertainty.
        self.x = self.x + np.dot(K, y)

        IminusKH = np.eye(4) - np.dot(K, self.H)
        self.P = np.dot(np.dot(IminusKH, self.P), IminusKH.T) + np.dot(np.dot(K, self.R), K.T)

        self.xPosterior = self.x.copy()
        self.PPosterior = self.P.copy()

        return self.xPosterior, self.PPosterior, self.y, self.K, self.S


# Display the predicted and estimated states at each time step.
def displayKFUpdates(time, xPrior, xPosterior, PPrior, PPosterior, Y, K) :
    "This Function Prints the Predictions and Estimation"
    print(f"TIME STEP {time}")
    print(f"PREDICTION {"\n"}x {xPrior[0, 0] : .4f}{"\n"}xVel {xPrior[1, 0] : .4f}{"\n"}y {xPrior[2, 0] : .4f}{"\n"}yVel {xPrior[3, 0] : .4f}")
    print(f"ESTIMATIONS {"\n"}x   {xPosterior[0, 0] : .4f}{"\n"}xVel  {xPosterior[1, 0] : .4f}{"\n"}y   {xPosterior[2, 0] : .4f}{"\n"}yVel {xPosterior[3, 0] : .4f}")
    print("_____________________________________________________________________________")


# Display the ground truth and the GPS measurements for comparison.
def displayValues(time, gpsValues, groundTruth) :
    
    print(f"DIMENSION {"\t"}GROUND TRUTH{"\t"}GPS READINGS")
    print(f"X {"\t"}{groundTruth[0] : .4f}     {"\t"}{gpsValues[0] : .4f}")
    print(f"Y {"\t"}{groundTruth[1] : .4f}     {"\t"}{gpsValues[1] : .4f}")
    print(f"Z {"\t"}{groundTruth[2] : .4f}     {"\t"}{gpsValues[2] : .4f}")
    print("_____________________________________________________________________________")


# Move the robot in a straight line or make it follow a circular path.
def moveRobot(leftM, rightM, angVelocity, inStraightLine = True, alpha = 0.5) :
    
    if inStraightLine :
        leftM.setVelocity(angVelocity)
        rightM.setVelocity(angVelocity)
    else :
        leftM.setVelocity(angVelocity + alpha)
        rightM.setVelocity(angVelocity - alpha)


# Plot the ground truth, GPS measurements, predictions, and posterior estimates with their uncertainties.
def plotPosition(
    time,
    groundTruth,
    measurements,
    predicted,
    estimated,
    predictedStd,
    estimatedStd,
    axisLabel="X position (m)"
):
    time = np.asarray(time)

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(
        time, groundTruth,
        "k--", label="Ground truth"
    )

    ax.scatter(
        time, measurements,
        facecolors="none", edgecolors="black",
        label="GPS measurements", zorder=3
    )

    ax.plot(
        time, predicted,
        color="orange", linestyle=":",
        label="Prediction"
    )

    ax.plot(
        time, estimated,
        color="tab:blue",
        label="Posterior estimate"
    )

    # Show the ±1 standard deviation range around the prediction and estimate.
    ax.fill_between(
        time,
        np.asarray(estimated) - np.asarray(estimatedStd),
        np.asarray(estimated) + np.asarray(estimatedStd),
        color="tab:blue", alpha=0.2,
        label="Estimate ±1σ"
    )

    ax.fill_between(
        time,
        np.asarray(predicted) - np.asarray(predictedStd),
        np.asarray(predicted) + np.asarray(predictedStd),
        color="orange", alpha=0.12,
        label="Prediction ±1σ"
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(axisLabel)
    ax.set_title(f"Kalman Filter — {axisLabel}")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    return fig, ax


# Plot the innovation and its uncertainty bounds to check the filter's consistency.
def plotResiduals(
    time,
    residual,
    innovationStd,
    axisLabel="X residual (m)"
):
    time = np.asarray(time)
    residual = np.asarray(residual)
    innovationStd = np.asarray(innovationStd)

    fig, ax = plt.subplots(figsize=(12, 5))

    ax.plot(
        time, residual,
        "ko-", markersize=3,
        label="Innovation"
    )

    ax.axhline(0, color="black", linewidth=1)

    # Check whether the innovation stays within the expected uncertainty bounds.
    for n in (1, 2, 3):
        ax.plot(
            time, n * innovationStd,
            linestyle="--", alpha=0.8,
            label=f"+{n}σ" if n == 1 else f"±{n}σ"
        )
        ax.plot(
            time, -n * innovationStd,
            linestyle="--", alpha=0.8
        )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(axisLabel)
    ax.set_title(f"Innovation Consistency — {axisLabel}")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    return fig, ax


if __name__ == "__main__" :
    # Supervisor Object
    robot = Supervisor()

    # Robot Node
    robotNode = robot.getSelf()

    # Translation Field instance
    translationField = robotNode.getField('translation')

    # Instances of Left and Right Motors
    rightMotor = robot.getDevice('motorRight')
    leftMotor = robot.getDevice('motorLeft')

    rightMotor.setPosition(float('inf'))
    leftMotor.setPosition(float('inf'))

    rightMotor.setVelocity(0.)
    leftMotor.setVelocity(0.0)

    # Instance of GPS Node
    gpsAccess = robot.getDevice('GPS')

    
    # Webots Simulation TimeStep
    timeStep = 64

    linearVelocity = .5
    wheelRadius = 0.05
    angularVelocity = linearVelocity/wheelRadius
    inStraightLine = True

    # Enable the GPS
    gpsAccess.enable(timeStep)

    # Standard deviations of State varaibles and measurements
    stdX = 5.
    stdY = 5.
    stdXVel = 10.
    stdYVel = 10.

    # Standard Deviation of GPS
    stdZX = 3.
    stdZY = 3.

    
    # Initialize the Kalman Filter with the required uncertainties and time step.
    MultivariateKF = KalmanFilter(stdX = stdX, stdY = stdY, stdXVel = stdXVel, stdYVel = stdYVel, stdZX = stdZX, stdZY = stdZY, stdAcc = .1, dt = timeStep/1000.)

    # Store the values at each time step so we can compare the filter's predictions and estimates later.
    TimeHistory = []

    groundTruthX = []
    groundTruthY = []

    GPSX = []
    GPSY = []

    predictedX = []
    predictedY = []

    predictedStdX = []
    predictedStdY = []

    residualX = []
    residualY = []

    estimatedX = []
    estimatedY = []

    estimatedStdX = []
    estimatedStdY = []

    inovStdX = []
    inovStdY = []

    while robot.step(timeStep) != -1:

        # Current simulation Time
        cTime = robot.getTime()

        # Ground Truth
        groundTruth = translationField.getSFVec3f()

        # Move The Robot
        moveRobot(leftMotor, rightMotor, angularVelocity, inStraightLine, alpha = 1.)

        # GPS Measurements
        positionFromGPS = gpsAccess.getValues()

        # Kalman Filter Predict
        xPrior, PPrior = MultivariateKF.predict()

        # Measurement
        z = np.array([[positionFromGPS[0]], [positionFromGPS[1]]])

        # Kalman filter update
        xPosterior, PPosterior, residual, kalmanGain, S = MultivariateKF.update(z)

        # display the Results on console
        displayValues(cTime, positionFromGPS, groundTruth)
        displayKFUpdates(cTime, xPrior, xPosterior, PPrior, PPosterior, residual, kalmanGain)

        # Store ground truth, GPS readings, predictions, estimates, and uncertainties for plotting.
        TimeHistory.append(cTime)

        groundTruthX.append(groundTruth[0])
        groundTruthY.append(groundTruth[1])

        GPSX.append(positionFromGPS[0])
        GPSY.append(positionFromGPS[1])

        predictedX.append(xPrior[0, 0])
        predictedY.append(xPrior[2, 0])

        estimatedX.append(xPosterior[0, 0])
        estimatedY.append(xPosterior[2, 0])

        predictedStdX.append(max(np.sqrt(PPrior[0, 0]), 0.))
        predictedStdY.append(max(np.sqrt(PPrior[2, 2]), 0.))

        estimatedStdX.append(max(np.sqrt(PPosterior[0, 0]), 0.))
        estimatedStdY.append(max(np.sqrt(PPosterior[2, 2]), 0.))

        residualX.append(residual[0, 0])
        residualY.append(residual[1, 0])

        inovStdX.append(max(np.sqrt(S[0, 0]), 0.))
        inovStdY.append(max(np.sqrt(S[1, 1]), 0.))

    # Save the collected data so the plots can be generated separately after the simulation.
    np.savez(
        "MultivariateKF_results.npz",
        time=TimeHistory,
        groundTruthX=groundTruthX,
        groundTruthY=groundTruthY,
        GPSX=GPSX,
        GPSY=GPSY,
        predictedX=predictedX,
        predictedY=predictedY,
        estimatedX=estimatedX,
        estimatedY=estimatedY,
        predictedStdX=predictedStdX,
        predictedStdY=predictedStdY,
        estimatedStdX=estimatedStdX,
        estimatedStdY=estimatedStdY,
        residualX=residualX,
        residualY=residualY,
        inovStdX=inovStdX,
        inovStdY=inovStdY
    )

    