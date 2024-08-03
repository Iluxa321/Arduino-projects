///////////////////////////////////////////////////////
// !!!! До офрмить подклоючение к плате
///////////////////////////////////////////////////////

let socket = new WebSocket("ws://192.168.1.3:81")
const slider = document.querySelector(".slider")
const settings = {
	addressBoard: 255,	// 0 - 254: 255 - не подключенн к плате
	connectDisconnet: 0,	// false/true 
	mode: 0,			// 0 - ожидание, 1 - PWM
	pwmArr: [0],		// 0 - 255
	numPwm: 1,
	timeUpdate: 100 	// 100 - 3000 мc
}

const recive_data = {
	boardState: 0,
	voltage: 0,
	current: 0
}

const graph = document.getElementById("Graphic");

const voltage_buffer = new Buffer(20)
const current_buffer = new Buffer(20)
const time_buffer = new TimeBuffer(20)


function Buffer(size) {
	this.arr = [...(function* (){
		for(let i = 0; i < size; i++) {
			yield 0
		}
	})()]
	this.update = (val) => {
		this.arr.shift()
		this.arr.push(val)
	}
}

function TimeBuffer(size) {
	this.arr = [...(function* (){
		for(let i = 0; i < size; i++) {
			yield 0
		}
	})()]
	this.sum = 0
	this.update = (time) => {
		this.sum = this.sum + time
		this.arr.shift()
		this.arr.push(this.sum)
	}
}


function defaultSettings(settings) {
	settings.addrBoard = 255
	settings.connectDisconnet = 0
	settings.mode = 0
	settings.pwmArr = [0]
	settings.numPwm = 1
	settings.timeUpdate = 100
}

///////////////////////////////////////////////////////
// Функции работающие с WebSocket
///////////////////////////////////////////////////////

function updateInfo(elem, text, status) {
	elem.innerHTML = text
	const info = elem.parentElement
	switch(status) {
		case 'CONNECTED': {
			info.classList.add("connected")
			info.classList.remove("disconnected")
			break
		}
		case 'DISCONNECTED': {
			info.classList.add("disconnected")
			info.classList.remove("connected")
			break
		}

	}
}

let connecting_to_server = new Promise(function(resolve, reject) {
	const info = document.querySelector(".info").querySelectorAll("p")[1];
	const text = info.innerHTML;
  let dot = "";
  setTimeout(function update_point() {
  	if(socket.readyState == 1){	// OPEN
  		resolve("Соединение установленно!");
  	}
  	else if(socket.readyState == 3){	// CLOSED
  		reject("Не удалось установить соединение");
  	} 
  	else{
  		dot = dot + ".";
  		if(dot.length > 4){
  			dot = "";
  		}
  		
  		info.innerHTML = text + "<strong> " + dot + "</strong>";
  		setTimeout(update_point, 500);
  	}
  	
  }, 500);
});

connecting_to_server.then(
  result => {
  	const info_text = document.querySelector(".info").querySelectorAll("p")[1];
  	info_text.innerHTML = result;
  	const info = document.querySelector(".info");
  	info.classList.add("connected");
  	info.classList.remove("disconnected");
	document.querySelector('.board').classList.remove('no-active');
	
  }, 
  error => {
  	const info_text = document.querySelector(".info").querySelectorAll("p")[1];
  	info_text.innerHTML = error;
  	const info = document.querySelector(".info");
  	info.classList.remove("connected");
  	info.classList.add("disconnected");
  }
);

socket.onclose = function(event) {
	const info_text = document.querySelector(".info").querySelectorAll("p")[1];
	info_text.innerHTML = "Не удалось установить соединение";
	const info = document.querySelector(".info");
	info.classList.remove("connected");
	info.classList.add("disconnected");
};


socket.onmessage = function (msg) {
	const m = JSON.parse(msg.data);
	recive_data.boardState = m.boardState
	recive_data.voltage = m.voltage
	recive_data.current = m.current
	// if(recive_data.boardState != 1) {
	// 	const boardStatus = document.querySelector("#board-status")
	// 	const info = boardStatus.parentElement
	// 	disconnectedFailure(boardStatus, info, "Не удалось установить соединение с платой")
	// }
	if(recive_data.boardState == 1) {
		if(settings.mode == 1) {
			const values = document.querySelectorAll('.mode1 > .display > .current-value')
			const [voltage, current] = [values[0], values[1]]
			voltage.querySelector('input').value = recive_data.voltage
			current.querySelector('input').value = recive_data.current
			voltage_buffer.update(Number(recive_data.voltage))
			current_buffer.update(Number(recive_data.current))
			time_buffer.update(settings.timeUpdate)
			plotUpdate(voltage_buffer.arr, current_buffer.arr, time_buffer.arr)
	
		}
	}
	

}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
// Board Menu
///////////////////////////////////////////////////////////////////////////////////

document.querySelectorAll(".visiable-field").forEach(element => {
	element.addEventListener("click", function(event) {
		const menu = this.parentElement.querySelector('.menu-slider-item');
		menu.classList.toggle('item-show')
	})
});



document.querySelectorAll(".menu-slider-item").forEach(element => {
	element.addEventListener("click", function(event) {
		this.classList.toggle("item-show");
		const field = this.parentElement.previousElementSibling.querySelector("p");	// visiable-field
		field.innerHTML = event.target.innerHTML;
		const addrBoard = event.target.getAttribute('value');
		field.setAttribute('value', addrBoard)
		settings.addressBoard = addrBoard;
	})
});


document.querySelector('.connected-board').addEventListener('click', function (event) {
	const boardStatus = document.querySelector("#board-status")
	const info = boardStatus.parentElement
	// Подключаение к плате и отколючение от платы 
	let value = this.getAttribute("value");
	if(value == 0) {
		value = 1
		this.previousElementSibling.classList.add('transparent-no-active')
		this.classList.add('transparent-no-active')
	}
	else {
		value = 0
		this.querySelector('p').innerHTML = 'Подключиться'
		this.previousElementSibling.classList.remove('transparent-no-active')
	}
	this.setAttribute("value", value)

	// Подключаемся или отлючаемся от платы
	settings.connectDisconnet = Boolean(value);
	if(settings.connectDisconnet) {
		let boardConnected = connectedToBoard(settings)
		boardConnected.then(
			resolve => {
				connectedSuccess(boardStatus, info, resolve)
				// boardStatus.innerHTML = resolve
				// info.classList.add("connected")
				// info.classList.remove("disconnected")
				// document.querySelector('.mode-menu').classList.remove('no-active')
				// const btn = document.querySelector('.connected-board')
				// btn.classList.remove('transparent-no-active')
				// btn.querySelector('p').innerHTML = 'Отключиться'

			},
			reject => {
				connectedFailure(boardStatus, info, reject)
				// boardStatus.innerHTML = reject
				// info.classList.remove("connected")
				// info.classList.add("disconnected")
				// document.querySelector('.mode-menu').classList.add('no-active')
			}
		)
	}
	else {
		disconnectedToBoard(settings)
		defaultSettings(settings)
		info.classList.add("no-active")
		info.classList.remove("connected")
		info.classList.remove("disconnected")
		document.querySelector('.mode-menu').classList.add('no-active')
		const elem = document.querySelector('.modes').children
		for(let i = 0; i < elem.length; i++) {
			elem[i].classList.remove('setting-show')	
		}
	}
	



})


function connectedSuccess (status, info, text) {
	status.innerHTML = text
	info.classList.add("connected")
	info.classList.remove("disconnected")
	document.querySelector('.mode-menu').classList.remove('no-active')
	const btn = document.querySelector('.connected-board')
	btn.classList.remove('transparent-no-active')
	btn.querySelector('p').innerHTML = 'Отключиться'
}

function connectedFailure (status, info, text) {
	status.innerHTML = text
	info.classList.remove("connected")
	info.classList.add("disconnected")
	document.querySelector('.mode-menu').classList.add('no-active')
	const connecte_btn = document.querySelector('.connected-board')
	connecte_btn.setAttribute("value", 0)
	connecte_btn.previousElementSibling.classList.remove('transparent-no-active')
	connecte_btn.classList.remove('transparent-no-active')
	
	
}

function disconnectedToBoard(settings) {
	const send_data = JSON.stringify(settings)
	socket.send(send_data)
}

function connectedToBoard(settings) {
	const send_data = JSON.stringify(settings)
	socket.send(send_data)
	return new Promise(function(resolve, reject) {
		recive_data.boardState = 0;
		const info = document.querySelector("#board-status");
		info.parentElement.classList.remove('no-active');
		info.innerHTML = 'Подключение'
		const text = info.innerHTML;
	  	let dot = "";
		setTimeout(function update_point() {
			if(recive_data.boardState == 1){		// Успешное подплючение к плате
				resolve("Соединение с платой установлено");
			}
			else if(recive_data.boardState == 2){	// Плата занята
				reject("Не удалось установить соединение с платой");
			} 
			else if(recive_data.boardState == 3) {	// Неудалось подключится к плате (В идеале это ошибки не должна возникать !!!)
				reject("Ошибка при подключении к плате");
			}
			else {
				dot = dot + ".";
				if(dot.length > 4){
					dot = "";
			}
				info.innerHTML = text + "<strong> " + dot + "</strong>";
				setTimeout(update_point, 500);
			}
		}, 500);
	});
}





///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// Modes
////////////////////////////////////////////////////////////////////////////////

document.querySelector('.mode-menu .menu-slider-item').addEventListener('click', function(event) {
	let num_mode = Number(event.target.getAttribute('value')) - 1
	const modes = document.querySelector('.modes').children
	for(let i = 0; i < modes.length; i++) {
		if(i == num_mode) {
			modes[i].classList.add('setting-show')
		}
		else {
			modes[i].classList.remove('setting-show')
		}
	}
})


document.querySelector('.save-button').addEventListener('click', function (event) {
	const mode_setting = this.parentElement.querySelector('.setting')
	const num_mode = Number(document.querySelector('.mode-menu .visiable-field > p').getAttribute('value'))
	let mode = 0
	let numPwm = 0
	let pwmArr = 0
	let timeUpdate = 1
	switch(num_mode) {
		case 1: {
			mode = 1
			numPwm = 1
			pwmArr = [Number(mode_setting.querySelector('.pwm_val p').innerHTML)]
			timeUpdate = Number(mode_setting.querySelector('.time input').value)
			break
		}
		case 2: {
			break
		}
	}
	settings.mode = mode
	settings.numPwm = numPwm
	settings.pwmArr = pwmArr
	settings.timeUpdate = timeUpdate
	const send_data = JSON.stringify(settings)
	socket.send(send_data)
})

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

slider.oninput = function (event) {
	const elem = document.querySelector(".pwm_val p");
	elem.innerHTML  = this.value;
}




window.addEventListener('load', function (event) {
	
	var trace1 = {
		mode: 'lines+markers',
		line: {shape: 'spline'},
		name: 'Напряжение',
		x: [],
		y: [],
		type: 'scatter'
	  };
	
	var trace2 = {
	  mode: 'lines+markers',
	  line: {shape: 'spline'},
	  name: 'Ток',
	  x: [],
	  y: [],
	  xaxis: 'x2',
  	  yaxis: 'y2',
	  type: 'scatter',
	  showarrow: true,
	  xanchor: 'left',
		yanchor: 'middle'
	};
	
	var layout = {
		grid: {rows: 2, columns: 1, pattern: 'independent',

		},
		height: 700,
		font: {
			size: 15,
		},
		title: 'Характеристики',
		paper_bgcolor: '#eee',
	  	xaxis: {
			title: 'Время, мс',
			showline: true,
			showgrid: true,
			zeroline: false,
			showticklabels: true,
			linecolor: 'rgb(204,204,204)',
			linewidth: 2,
			autotick: true,
			ticks: 'outside',
			tickcolor: 'rgb(204,204,204)',
			tickwidth: 2,
			ticklen: 5,
			tickfont: {
			family: 'Arial',
			size: 12,
			color: 'rgb(82, 82, 82)'
			},
			
	  },
		yaxis: {
			title: 'U, B',
			showgrid: true,
			showline: true,
			zeroline: false,
			showticklabels: true,
			linecolor: 'rgb(204,204,204)',
			linewidth: 2,
			autotick: true,
			ticks: 'outside',
			tickcolor: 'rgb(204,204,204)',
			tickfont: {
				family: 'Arial',
				size: 12,
				color: 'rgb(82, 82, 82)'
			}
		},
		xaxis2: {
			title: 'Время, мс',
			showline: true,
			showgrid: true,
			zeroline: false,
			showticklabels: true,
			linecolor: 'rgb(204,204,204)',
			linewidth: 2,
			autotick: true,
			ticks: 'outside',
			tickcolor: 'rgb(204,204,204)',
			tickwidth: 2,
			ticklen: 5,
			tickfont: {
			family: 'Arial',
			size: 12,
			color: 'rgb(82, 82, 82)'
			}
		},
		yaxis2: {
			title: 'I, A',
			showgrid: true,
			showline: true,
			zeroline: false,
			showticklabels: true,
			linecolor: 'rgb(204,204,204)',
			linewidth: 2,
			autotick: true,
			ticks: 'outside',
			tickcolor: 'rgb(204,204,204)',
			tickfont: {
				family: 'Arial',
				size: 12,
				color: 'rgb(82, 82, 82)'
			}
		},
	};
	
	var data = [trace1, trace2];
	Plotly.newPlot(graph, data, layout);
})


function plotUpdate(voltage, current, time) {
	var update = {
		'x': [time, time],
		'y': [voltage, current]		
	}
	Plotly.update(graph, update)

}
