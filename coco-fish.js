let flock;
let letterPositions = [
    [54,90],
    [110, 90],
    [183, 90],
    [242, 90],
    [316, 90],
    [388, 90],
    [460, 90],
    [54, 170],
    [116, 170],
    [189, 170],
    [250, 170],
    [322, 170],
    [394, 170],
    [462, 170]
];
let stencilWidth = 514.0;
let stencilHeight = 270.0;
let startTime;
let capturer;

function setup() {
    createCanvas(windowWidth, windowHeight);

    // Create images
    const images = {};
    Array.from("CONUTSDI").map((c) => {
        images[c] = loadImage(`/coco-sans/${c}.svg`);
    });

    flock = new Flock();

    const letterDimension = 130 * (width / stencilWidth);

    // Add an initial set of boids into the system
    Array.from("COCONUTSTUDIOS").map((c, j) => {
        const letterDestination = 
            [(width / stencilWidth) * letterPositions[j][0] - 50, 
             (height / stencilHeight) * letterPositions[j][1] - letterDimension / 2];
       
        const initialX = Math.random() * width;
        const initialY = Math.random() * height;

        let b = new Boid(initialX, initialY, images[c], letterDimension, letterDimension, letterDestination);
        
        flock.addBoid(b);
    });

    startTime = new Date().getTime();
    fullscreen();
    
    // Create a capturer that exports a WebM video
    // setTimeout(() => {
    //     capturer = new CCapture( { format: 'webm', framerate: 25 } ); 
    //     capturer.start();
    // }, 1000);
}

function draw() {
    background(51);

    flock.run();

    // if (capturer && canvas) {
    //     const canvas = document.getElementById("defaultCanvas0");
    //     capturer.capture(canvas);
    //     if (frameCount > 300) {
    //         capturer.stop();
    //         capturer.save();
    //     }
    // 
}

// Start growing after 7 seconds
function dilation() {
    const currentTime = new Date().getTime();
    const timeFactor = Math.min(Math.max(0.0, currentTime - startTime - 7000) / 7000.0, 1.0);
    return 0.5 + 0.5 * timeFactor;
}

// Start steering after 5 seconds
function timeWeighting() {
    const currentTime = new Date().getTime();
    return Math.min(Math.max(0.0, currentTime - startTime - 5000) / 9000.0, 1.0);
}

// Rotate and draw an image
// Adapted from https://stackoverflow.com/questions/45388765/how-to-rotate-image-in-p5-js
function rotateAndDrawImage(img, x, y, width, height, radians) {
    // const d = dilation();
    const d = 1;

    imageMode(CENTER);
    translate(x + d * width / 2, y + d * height / 2);
    rotate(radians);
    image(img, 0, 0, d * width, d * height);
    rotate(- radians);
    translate(-(x + d * width / 2), -(y + d * height / 2));
    imageMode(CORNER);
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Flock object
// Does very little, simply manages the array of all the boids


function Flock() {
    // An array for all the boids
    this.boids = []; // Initialize the array
}

Flock.prototype.run = function () {
    for (let i = 0; i < this.boids.length; i++) {
        this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
    }
}

Flock.prototype.addBoid = function (b) {
    this.boids.push(b);
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x, y, img, width, height, destination) {
    this.img = img;
    this.width = width;
    this.height = height;
    this.acceleration = createVector(0, 0);
    this.velocity = createVector(random(-3, 3), random(-3, 3));
    this.position = createVector(x, y);
    this.r = 70;
    this.maxspeed = 5;    // Maximum speed
    this.maxforce = 0.05; // Maximum steering force
    this.destination = destination;
}

Boid.prototype.run = function (boids) {
    this.flock(boids);
    this.update();
    this.borders();
    this.render();
}

Boid.prototype.applyForce = function (force) {
    // We could add mass here if we want A = F / M
    this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function (boids) {
    let sep = this.separate(boids);   // Separation
    let ali = this.align(boids);      // Alignment
    let coh = this.cohesion(boids);   // Cohesion
    let trav = this.travel();

    // Weight these forces based on time
    let w = timeWeighting();
    let negw = 1 - w;

    sep.mult(1 * negw);
    ali.mult(1 * negw);
    coh.mult(1 * negw);
    trav.mult(w);

    // Add the force vectors to acceleration
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
    this.applyForce(trav);
}

// Method to update location
Boid.prototype.update = function () {
    // Update velocity
    this.velocity.add(this.acceleration);
    // Limit speed
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    // Reset accelertion to 0 each cycle
    this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function (target) {
    let desired = p5.Vector.sub(target, this.position);  // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(this.maxspeed);
    // Steering = Desired minus Velocity
    let steer = p5.Vector.sub(desired, this.velocity);
    steer.limit(this.maxforce);  // Limit to maximum steering force
    return steer;
}

Boid.prototype.render = function () {
    let theta = this.velocity.heading() + radians(90);
    rotateAndDrawImage(this.img, this.position.x, this.position.y, this.width, this.height, theta);
}

// Wraparound
Boid.prototype.borders = function () {
    if (this.position.x < -this.r) this.position.x = width + this.r;
    if (this.position.y < -this.r) this.position.y = height + this.r;
    if (this.position.x > width + this.r) this.position.x = -this.r;
    if (this.position.y > height + this.r) this.position.y = -this.r;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function (boids) {
    let desiredseparation = 40.0;
    let steer = createVector(0, 0);
    let count = 0;
    // For every boid in the system, check if it's too close
    for (let i = 0; i < boids.length; i++) {
        let d = p5.Vector.dist(this.position, boids[i].position);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((d > 0) && (d < desiredseparation)) {
            // Calculate vector pointing away from neighbor
            let diff = p5.Vector.sub(this.position, boids[i].position);
            diff.normalize();
            diff.div(d);        // Weight by distance
            steer.add(diff);
            count++;            // Keep track of how many
        }
    }
    // Average -- divide by how many
    if (count > 0) {
        steer.div(count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer.mult(this.maxspeed);
        steer.sub(this.velocity);
        steer.limit(this.maxforce);
    }
    return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function (boids) {
    let neighbordist = 50;
    let sum = createVector(0, 0);
    let count = 0;
    for (let i = 0; i < boids.length; i++) {
        let d = p5.Vector.dist(this.position, boids[i].position);
        if ((d > 0) && (d < neighbordist)) {
            sum.add(boids[i].velocity);
            count++;
        }
    }
    if (count > 0) {
        sum.div(count);
        sum.normalize();
        sum.mult(this.maxspeed);
        let steer = p5.Vector.sub(sum, this.velocity);
        steer.limit(this.maxforce);
        return steer;
    } else {
        return createVector(0, 0);
    }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function (boids) {
    let neighbordist = 50;
    let sum = createVector(0, 0);   // Start with empty vector to accumulate all locations
    let count = 0;
    for (let i = 0; i < boids.length; i++) {
        let d = p5.Vector.dist(this.position, boids[i].position);
        if ((d > 0) && (d < neighbordist)) {
            sum.add(boids[i].position); // Add location
            count++;
        }
    }
    if (count > 0) {
        sum.div(count);
        return this.seek(sum);  // Steer towards the location
    } else {
        return createVector(0, 0);
    }
}

Boid.prototype.travel = function () {
    let dest = createVector(this.destination[0], this.destination[1]);
    let travel = p5.Vector.sub(dest, this.position);  // A vector pointing from the location to the target
    let distance = travel.mag();
    let steer = travel.normalize();        
    
    const speed = this.velocity.mag();
    const stopDistance = Math.pow(speed, 2) / (2 * this.maxforce);
    if (distance > stopDistance) {
        steer.mult(this.maxspeed);
        steer.sub(this.velocity);
    } else {
        steer.mult(Math.min(0.0, this.velocity.mag() - this.maxforce));
    }
    steer.limit(this.maxforce);        

    return steer;
}
